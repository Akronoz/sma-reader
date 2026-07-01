#!/usr/bin/env python3
"""
Envía lecturas de la planta SMA al VPS (push desde la RPi).

Variables de entorno:
  SMA_VPS_URL     Endpoint del servidor (ej. https://tu-vps/api/v1/snapshots)
  SMA_API_KEY     Clave compartida (header X-API-Key)
  SMA_INTERVAL    Segundos entre envíos (default: 60)
  SMA_MODBUS_HOST IP del Data Manager (default: 192.168.1.101)
  SMA_SLOW        "1" para modo red lenta (mismos valores que sma_plant.py --slow)

Uso:
  python sma_push.py --once          # prueba: una lectura y un envío
  python sma_push.py                 # bucle continuo
  python sma_push.py -w -i 30        # equivalente, intervalo 30 s
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
        if ok:
            print(f"[{payload['sent_at']}] OK ({detail}) | Producción={prod} | Consumo={cons}")
        else:
            print(
                f"[{payload['sent_at']}] ERROR: {detail} (último snapshot en {CACHE_FILE})",
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
        print("Falta SMA_VPS_URL o --vps-url", file=sys.stderr)
        return 1
    if not config["api_key"]:
        print("Falta SMA_API_KEY o --api-key", file=sys.stderr)
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

    loop = args.watch or not args.once
    if loop:
        print(f"Enviando a {config['vps_url']} cada {config['interval']}s (Ctrl+C para salir)")
        try:
            while True:
                push_once(reader, config, verbose=not args.quiet)
                time.sleep(config["interval"])
        except KeyboardInterrupt:
            print("\nDetenido.")
        return 0

    return 0 if push_once(reader, config, verbose=not args.quiet) else 1


if __name__ == "__main__":
    raise SystemExit(main())