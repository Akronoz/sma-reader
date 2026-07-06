#!/usr/bin/env python3
"""
Push SMA plant readings to the VPS (push from the RPi).

Environment variables:
  SMA_VPS_URL     Server endpoint (e.g. https://your-vps/api/v1/snapshots)
  SMA_API_KEY     Shared key (X-API-Key header)
  SMA_INTERVAL    Seconds between pushes (default: 60)
  SMA_MODBUS_HOST Data Manager IP (default: 192.168.1.101)
  SMA_SLOW        "1" for slow network mode (same values as sma_plant.py --slow)
  SMA_STALE_CHECK "0" to disable stale power meter detection

Usage:
  python sma_push.py --once          # test: one read and one push
  python sma_push.py                 # continuous loop
  python sma_push.py -w -i 30        # equivalent, 30 s interval
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

from sma_plant import (
    DEFAULT_DELAY,
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_RETRIES,
    DEFAULT_TIMEOUT,
    INVERTER_UNIT,
    METER_UNIT,
    SLOW_DELAY,
    SLOW_RETRIES,
    SLOW_TIMEOUT,
    SmaPlantReader,
    snapshot_to_dict,
)

DEFAULT_INTERVAL = 60.0
CACHE_FILE = Path(__file__).with_name(".sma_push_cache.json")
ENV_FILE = Path(__file__).with_name(".env")


def load_dotenv(path: Path = ENV_FILE) -> None:
    if not path.is_file():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    return default if not raw else float(raw)


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    return default if not raw else int(raw)


def load_config() -> dict:
    slow = os.environ.get("SMA_SLOW", "").lower() in {"1", "true", "yes"}
    return {
        "vps_url": os.environ.get("SMA_VPS_URL", "").strip(),
        "api_key": os.environ.get("SMA_API_KEY", "").strip(),
        "interval": _env_float("SMA_INTERVAL", DEFAULT_INTERVAL),
        "host": os.environ.get("SMA_MODBUS_HOST", DEFAULT_HOST),
        "port": _env_int("SMA_MODBUS_PORT", DEFAULT_PORT),
        "inverter_unit": _env_int("SMA_INVERTER_UNIT", INVERTER_UNIT),
        "meter_unit": _env_int("SMA_METER_UNIT", METER_UNIT),
        "timeout": SLOW_TIMEOUT if slow else _env_float("SMA_MODBUS_TIMEOUT", DEFAULT_TIMEOUT),
        "delay": SLOW_DELAY if slow else _env_float("SMA_MODBUS_DELAY", DEFAULT_DELAY),
        "retries": SLOW_RETRIES if slow else _env_int("SMA_MODBUS_RETRIES", DEFAULT_RETRIES),
        "stale_check": os.environ.get("SMA_STALE_CHECK", "1").lower() not in {"0", "false", "no"},
    }


def post_snapshot(url: str, api_key: str, payload: dict) -> tuple[bool, str]:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "X-API-Key": api_key,
            "User-Agent": "sma-push/1.0",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            response.read()
            return True, f"HTTP {response.status}"
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")[:300]
        return False, f"HTTP {exc.code}: {detail}"
    except urllib.error.URLError as exc:
        return False, str(exc.reason)
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)


def push_once(reader: SmaPlantReader, config: dict, verbose: bool = True) -> bool:
    t0 = time.monotonic()
    snap = reader.read_snapshot()
    payload = snapshot_to_dict(snap, elapsed_s=time.monotonic() - t0)
    payload["sent_at"] = time.strftime("%Y-%m-%d %H:%M:%S")

    ok, detail = post_snapshot(config["vps_url"], config["api_key"], payload)
    CACHE_FILE.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    if verbose:
        prod = snap.inverter_power.formatted()
        cons = snap.site_consumption.formatted()
        meter_note = " | Stale power meter" if snap.meter_stale else ""
        if ok:
            print(
                f"[{payload['sent_at']}] OK ({detail}) | Production={prod} | Consumption={cons}{meter_note}"
            )
        else:
            print(
                f"[{payload['sent_at']}] ERROR: {detail} (last snapshot in {CACHE_FILE})",
                file=sys.stderr,
            )
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Envía lecturas SMA al VPS")
    parser.add_argument("--vps-url", help="Sobrescribe SMA_VPS_URL")
    parser.add_argument("--api-key", help="Sobrescribe SMA_API_KEY")
    parser.add_argument("-i", "--interval", type=float, help="Segundos entre envíos")
    parser.add_argument("-w", "--watch", action="store_true", help="Bucle continuo (default si no --once)")
    parser.add_argument("--once", action="store_true", help="Una lectura y salir")
    parser.add_argument("-q", "--quiet", action="store_true")
    args = parser.parse_args()
    load_dotenv()

    config = load_config()
    if args.vps_url:
        config["vps_url"] = args.vps_url
    if args.api_key:
        config["api_key"] = args.api_key
    if args.interval is not None:
        config["interval"] = args.interval

    if not config["vps_url"]:
        print("Missing SMA_VPS_URL or --vps-url", file=sys.stderr)
        return 1
    if not config["api_key"]:
        print("Missing SMA_API_KEY or --api-key", file=sys.stderr)
        return 1

    reader = SmaPlantReader(
        host=config["host"],
        port=config["port"],
        inverter_unit=config["inverter_unit"],
        meter_unit=config["meter_unit"],
        timeout=config["timeout"],
        delay=config["delay"],
        retries=config["retries"],
    )
    reader.stale_check = config["stale_check"]

    loop = args.watch or not args.once
    if loop:
        print(f"Sending to {config['vps_url']} every {config['interval']}s (Ctrl+C to exit)")
        try:
            while True:
                push_once(reader, config, verbose=not args.quiet)
                time.sleep(config["interval"])
        except KeyboardInterrupt:
            print("\nStopped.")
        return 0

    return 0 if push_once(reader, config, verbose=not args.quiet) else 1


if __name__ == "__main__":
    raise SystemExit(main())