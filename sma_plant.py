#!/usr/bin/env python3
"""
Modbus TCP reader for SMA plant with Data Manager.

Verified configuration on local network:
  - Data Manager: 192.168.1.101:502
  - 100 kW inverter: unit_id=10
  - Power meter:     unit_id=11

Usage:
  python sma_plant.py                  # quick read (~3-10 s)
  python sma_plant.py --slow           # if inverter WiFi is unstable
  python sma_plant.py -w -i 10         # monitor every 10 s

Push to VPS (separate script):
  python sma_push.py --once
  python sma_push.py -w -i 60
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# --- Default configuration ---
DEFAULT_HOST = "192.168.1.101"
DEFAULT_PORT = 502
INVERTER_UNIT = 10
METER_UNIT = 11

# By default: block reads, few requests (typically 2-8 s on normal LAN/WiFi).
# If it fails due to slow network, use: --slow
DEFAULT_TIMEOUT = 5.0
DEFAULT_DELAY = 0.15
DEFAULT_RETRIES = 1
SLOW_TIMEOUT = 20.0
SLOW_DELAY = 1.0
SLOW_RETRIES = 3

NAN_S32 = 0x80000000
NAN_U32 = 0xFFFFFFFF

METER_STALE_ERROR = "Power meter not updating (stale data)"
STALE_PROBE_INTERVAL_S = 1.0
STALE_INVERTER_DELTA_W = 50
STALE_MIN_EXPORT_KW = 0.5
STALE_MIN_INVERTER_W = 100

RegisterDef = tuple[int, str, float, str, str]  # addr, dtype, scale, unit, label
MeterFingerprint = tuple[int, ...]
BlockDef = tuple[int, int, list[tuple[int, str, float, str, str]]]
# block: (start_addr, register_count, [(addr, dtype, scale, unit, label), ...])


@dataclass
class Reading:
    label: str
    value: float | None = None
    unit: str = ""
    error: str | None = None

    def formatted(self) -> str:
        if self.error:
            return f"ERROR: {self.error}"
        if self.value is None:
            return "N/A"
        v = self.value
        u = self.unit
        if u == "W" and abs(v) >= 1000:
            return f"{v / 1000:.2f} kW"
        if u == "kW":
            return f"{v:.2f} kW"
        if u == "kVA":
            return f"{v:.2f} kVA"
        if u == "V":
            return f"{v:.1f} V"
        if u == "A":
            return f"{v:.2f} A"
        if u == "Hz":
            return f"{v:.2f} Hz"
        return f"{v:.3f} {u}".rstrip("0").rstrip(".")


@dataclass
class PlantSnapshot:
    timestamp: str
    host: str = DEFAULT_HOST
    inverter_unit: int = INVERTER_UNIT
    meter_unit: int = METER_UNIT
    inverter_power: Reading = field(default_factory=lambda: Reading("Potencia inversor", unit="W"))
    meter_import: Reading = field(default_factory=lambda: Reading("Consumo red (import)", unit="kW"))
    meter_export: Reading = field(default_factory=lambda: Reading("Exportación red", unit="kW"))
    meter_balance: Reading = field(default_factory=lambda: Reading("Balance en vatímetro", unit="kW"))
    site_consumption: Reading = field(default_factory=lambda: Reading("Consumo instalación", unit="kW"))
    meter_apparent_total: Reading = field(
        default_factory=lambda: Reading("Potencia aparente total", unit="kVA")
    )
    meter_frequency: Reading = field(default_factory=lambda: Reading("Frecuencia", unit="Hz"))
    meter_export_phases: list[Reading] = field(default_factory=list)
    meter_apparent_phases: list[Reading] = field(default_factory=list)
    meter_voltage: list[Reading] = field(default_factory=list)
    dc_strings: list[Reading] = field(default_factory=list)
    meter_stale: bool = False
    meter_stale_reason: str | None = None


def decode(registers: list[int], dtype: str) -> int | None:
    if dtype == "s32":
        value = (registers[0] << 16) | registers[1]
        if value == NAN_S32:
            return None
        return value - 4_294_967_296 if value > 2_147_483_647 else value
    value = (registers[0] << 16) | registers[1]
    if value == NAN_U32:
        return None
    return value


# Inverter (unit 10): verified AC power
INVERTER_POWER: RegisterDef = (30775, "s32", 1, "W", "Potencia AC")

# Strings/DC: not available on this 100 kW inverter via Data Manager (Modbus error 4).
# Only attempted with --try-strings (1 retry, does not extend the normal read time).
INVERTER_STRING_REGISTERS: list[RegisterDef] = [
    (30769, "s32", 0.001, "A", "DC corriente MPPT1"),
    (30771, "s32", 0.01, "V", "DC tensión MPPT1"),
    (30773, "s32", 1, "W", "DC potencia MPPT1"),
    (30957, "s32", 0.001, "A", "DC corriente MPPT2"),
    (30959, "s32", 0.01, "V", "DC tensión MPPT2"),
    (30961, "s32", 1, "W", "DC potencia MPPT2"),
    (31793, "s32", 0.001, "A", "String 1 corriente"),
    (31795, "s32", 0.001, "A", "String 2 corriente"),
]

# Power meter (unit 11): use totals for consumption/balance (reliable registers).
METER_CORE_REGISTERS: list[RegisterDef] = [
    (30865, "s32", 1 / 1000, "kW", "Importación red (TotWIn)"),
    (30867, "s32", 1 / 1000, "kW", "Exportación red (TotWOut)"),
    (31455, "s32", 1 / 1000, "kVA", "Potencia aparente total (TotVA)"),
    (31447, "u32", 0.01, "Hz", "Frecuencia"),
]

# Per-phase detail (optional with --phases). Sum of L1+L2+L3 export = TotWOut.
# Currents 31435×V ≈ phase VA, NOT active power exported per phase.
METER_PHASE_REGISTERS: list[RegisterDef] = [
    (31259, "u32", 1 / 1000, "kW", "Exportación activa L1"),
    (31261, "u32", 1 / 1000, "kW", "Exportación activa L2"),
    (31263, "u32", 1 / 1000, "kW", "Exportación activa L3"),
    (31441, "s32", 1 / 1000, "kVA", "Potencia aparente L1"),
    (31443, "s32", 1 / 1000, "kVA", "Potencia aparente L2"),
    (31445, "s32", 1 / 1000, "kVA", "Potencia aparente L3"),
    (31253, "u32", 0.01, "V", "Tensión fase L1"),
    (31255, "u32", 0.01, "V", "Tensión fase L2"),
    (31257, "u32", 0.01, "V", "Tensión fase L3"),
]


class SmaPlantReader:
    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        inverter_unit: int = INVERTER_UNIT,
        meter_unit: int = METER_UNIT,
        timeout: float = DEFAULT_TIMEOUT,
        delay: float = DEFAULT_DELAY,
        retries: int = DEFAULT_RETRIES,
    ) -> None:
        self.host = host
        self.port = port
        self.inverter_unit = inverter_unit
        self.meter_unit = meter_unit
        self.timeout = timeout
        self.delay = delay
        self.retries = retries
        self.try_strings = False
        self.show_phases = False
        self.stale_check = True
        self._meter_fp: MeterFingerprint | None = None
        self._meter_fp_repeat = 0
        self._inv_raws_since_meter_fp: set[int] = set()

    def _pause(self) -> None:
        if self.delay > 0:
            time.sleep(self.delay)

    def _read(
        self, client: ModbusTcpClient, unit_id: int, addr: int, dtype: str
    ) -> tuple[int | None, str | None]:
        try:
            response = client.read_holding_registers(
                address=addr, count=2, device_id=unit_id
            )
            if response.isError():
                return None, str(response)
            return decode(response.registers, dtype), None
        except ModbusException as exc:
            return None, str(exc)
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

    def _read_register_words(
        self, client: ModbusTcpClient, unit_id: int, addr: int, count: int = 2
    ) -> tuple[int, ...] | None:
        try:
            response = client.read_holding_registers(
                address=addr, count=count, device_id=unit_id
            )
            if response.isError():
                return None
            return tuple(response.registers)
        except (ModbusException, Exception):  # noqa: BLE001
            return None

    def _read_inverter_power_raw(self, client: ModbusTcpClient) -> int | None:
        words = self._read_register_words(client, self.inverter_unit, 30775)
        return None if words is None else decode(list(words), "s32")

    def _read_meter_fingerprint(self, client: ModbusTcpClient) -> MeterFingerprint | None:
        parts: list[int] = []
        for addr in (30865, 30867, 31447, 31455):
            words = self._read_register_words(client, self.meter_unit, addr)
            if words is None:
                return None
            parts.extend(words)
        return tuple(parts)

    def _check_meter_stale(
        self,
        inv_raw_start: int | None,
        inv_raw_end: int | None,
        meter_fp_start: MeterFingerprint | None,
        meter_fp_end: MeterFingerprint | None,
        inverter_w: float | None,
        export_kw: float | None,
        import_kw: float | None,
    ) -> str | None:
        if (
            meter_fp_start is not None
            and meter_fp_end is not None
            and meter_fp_start == meter_fp_end
            and inv_raw_start is not None
            and inv_raw_end is not None
            and abs(inv_raw_end - inv_raw_start) >= STALE_INVERTER_DELTA_W
        ):
            return METER_STALE_ERROR

        inv = inverter_w or 0.0
        exp = export_kw or 0.0
        imp = import_kw or 0.0
        if inv < STALE_MIN_INVERTER_W and (
            exp >= STALE_MIN_EXPORT_KW or imp >= STALE_MIN_EXPORT_KW
        ):
            return METER_STALE_ERROR

        if meter_fp_end is not None:
            if meter_fp_end == self._meter_fp:
                self._meter_fp_repeat += 1
            else:
                self._meter_fp = meter_fp_end
                self._meter_fp_repeat = 1
                self._inv_raws_since_meter_fp = set()
            if inv_raw_end is not None:
                self._inv_raws_since_meter_fp.add(inv_raw_end)
            if self._meter_fp_repeat >= 2 and len(self._inv_raws_since_meter_fp) >= 2:
                return METER_STALE_ERROR

        return None

    def _read_with_retries(
        self,
        client: ModbusTcpClient,
        unit_id: int,
        addr: int,
        dtype: str,
        retries: int,
    ) -> tuple[int | None, str | None]:
        last_err = "Unknown error"
        for attempt in range(retries + 1):
            raw, err = self._read(client, unit_id, addr, dtype)
            if err is None:
                return raw, None
            last_err = err
            if attempt < retries:
                time.sleep(self.delay * 0.5)
        return None, last_err

    def _read_registers(
        self,
        unit_id: int,
        registers: list[RegisterDef],
        retries: int | None = None,
    ) -> dict[str, Reading]:
        effective_retries = self.retries if retries is None else retries
        results: dict[str, Reading] = {}
        client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout, retries=1)
        if not client.connect():
            for _, _, _, unit, label in registers:
                results[label] = Reading(label, unit=unit, error="No connection")
            return results

        try:
            for addr, dtype, scale, unit, label in registers:
                time.sleep(self.delay)
                raw, err = self._read_with_retries(
                    client, unit_id, addr, dtype, effective_retries
                )
                if err:
                    results[label] = Reading(label, unit=unit, error=err)
                elif raw is None:
                    results[label] = Reading(label, unit=unit)
                else:
                    results[label] = Reading(label, value=raw * scale, unit=unit)
        finally:
            client.close()
            time.sleep(0.5)

        return results

    def _read_block(
        self,
        client: ModbusTcpClient,
        unit_id: int,
        start_addr: int,
        count: int,
        fields: list[tuple[int, str, float, str, str]],
        retries: int,
    ) -> dict[str, Reading]:
        results: dict[str, Reading] = {}
        last_err = "Unknown error"
        raw_block: list[int] | None = None

        for attempt in range(retries + 1):
            self._pause()
            try:
                response = client.read_holding_registers(
                    address=start_addr, count=count, device_id=unit_id
                )
                if response.isError():
                    last_err = str(response)
                    time.sleep(self.delay * (attempt + 1))
                    continue
                raw_block = response.registers
                break
            except ModbusException as exc:
                last_err = str(exc)
            except Exception as exc:  # noqa: BLE001
                last_err = str(exc)
            time.sleep(self.delay * (attempt + 1))

        if raw_block is None:
            for _, _, _, unit, label in fields:
                results[label] = Reading(label, unit=unit, error=last_err)
            return results

        for addr, dtype, scale, unit, label in fields:
            offset = addr - start_addr
            if offset < 0 or offset + 2 > len(raw_block):
                results[label] = Reading(label, unit=unit, error="Out of block")
                continue
            regs = raw_block[offset : offset + 2]
            raw = decode(regs, dtype)
            if raw is None:
                results[label] = Reading(label, unit=unit)
            else:
                results[label] = Reading(label, value=raw * scale, unit=unit)
        return results

    def _read_snapshot_fast(
        self,
        probe_strings: bool,
        probe_phases: bool,
    ) -> tuple[
        dict[str, Reading],
        dict[str, Reading],
        int | None,
        int | None,
        MeterFingerprint | None,
        MeterFingerprint | None,
    ]:
        inv: dict[str, Reading] = {}
        meter: dict[str, Reading] = {}
        inv_raw_start: int | None = None
        inv_raw_end: int | None = None
        meter_fp_start: MeterFingerprint | None = None
        meter_fp_end: MeterFingerprint | None = None

        client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout, retries=0)
        if not client.connect():
            inv["Potencia AC"] = Reading("Potencia AC", unit="W", error="No connection")
            for _, _, _, unit, label in METER_CORE_REGISTERS:
                meter[label] = Reading(label, unit=unit, error="No connection")
            return inv, meter, None, None, None, None

        try:
            inv_raw_start = self._read_inverter_power_raw(client)

            # Inverter: 1 request
            inv.update(
                self._read_block(
                    client,
                    self.inverter_unit,
                    30775,
                    2,
                    [INVERTER_POWER],
                    self.retries,
                )
            )

            if probe_strings:
                for reg in INVERTER_STRING_REGISTERS:
                    self._pause()
                    raw, err = self._read_with_retries(
                        client, self.inverter_unit, reg[0], reg[1], 0
                    )
                    label = reg[4]
                    if err:
                        inv[label] = Reading(label, unit=reg[3], error=err)
                    elif raw is None:
                        inv[label] = Reading(label, unit=reg[3])
                    else:
                        inv[label] = Reading(label, value=raw * reg[2], unit=reg[3])

            # Power meter: 3 requests (TotWIn+TotWOut, Hz, TotVA)
            meter.update(
                self._read_block(
                    client,
                    self.meter_unit,
                    30865,
                    4,
                    [
                        (30865, "s32", 1 / 1000, "kW", "Importación red (TotWIn)"),
                        (30867, "s32", 1 / 1000, "kW", "Exportación red (TotWOut)"),
                    ],
                    self.retries,
                )
            )
            meter.update(
                self._read_block(
                    client,
                    self.meter_unit,
                    31447,
                    2,
                    [(31447, "u32", 0.01, "Hz", "Frecuencia")],
                    self.retries,
                )
            )
            meter.update(
                self._read_block(
                    client,
                    self.meter_unit,
                    31455,
                    2,
                    [(31455, "s32", 1 / 1000, "kVA", "Potencia aparente total (TotVA)")],
                    self.retries,
                )
            )

            meter_fp_start = self._read_meter_fingerprint(client)
            if self.stale_check:
                time.sleep(STALE_PROBE_INTERVAL_S)
                inv_raw_end = self._read_inverter_power_raw(client)
                meter_fp_end = self._read_meter_fingerprint(client)

            if probe_phases:
                meter.update(
                    self._read_block(
                        client,
                        self.meter_unit,
                        31253,
                        12,
                        [
                            (31253, "u32", 0.01, "V", "Tensión fase L1"),
                            (31255, "u32", 0.01, "V", "Tensión fase L2"),
                            (31257, "u32", 0.01, "V", "Tensión fase L3"),
                            (31259, "u32", 1 / 1000, "kW", "Exportación activa L1"),
                            (31261, "u32", 1 / 1000, "kW", "Exportación activa L2"),
                            (31263, "u32", 1 / 1000, "kW", "Exportación activa L3"),
                        ],
                        self.retries,
                    )
                )
                meter.update(
                    self._read_block(
                        client,
                        self.meter_unit,
                        31441,
                        6,
                        [
                            (31441, "s32", 1 / 1000, "kVA", "Potencia aparente L1"),
                            (31443, "s32", 1 / 1000, "kVA", "Potencia aparente L2"),
                            (31445, "s32", 1 / 1000, "kVA", "Potencia aparente L3"),
                        ],
                        self.retries,
                    )
                )
        finally:
            client.close()

        return inv, meter, inv_raw_start, inv_raw_end, meter_fp_start, meter_fp_end

    def read_snapshot(
        self,
        try_strings: bool | None = None,
        show_phases: bool | None = None,
    ) -> PlantSnapshot:
        probe_strings = self.try_strings if try_strings is None else try_strings
        probe_phases = self.show_phases if show_phases is None else show_phases
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        snap = PlantSnapshot(
            timestamp=ts,
            host=self.host,
            inverter_unit=self.inverter_unit,
            meter_unit=self.meter_unit,
        )

        inv, meter, inv_raw_start, inv_raw_end, meter_fp_start, meter_fp_end = (
            self._read_snapshot_fast(probe_strings, probe_phases)
        )

        if "Potencia AC" in inv:
            snap.inverter_power = inv["Potencia AC"]

        for label in ("Importación red (TotWIn)", "Exportación red (TotWOut)"):
            if label in meter:
                if "Import" in label:
                    snap.meter_import = meter[label]
                else:
                    snap.meter_export = meter[label]

        imp = snap.meter_import.value or 0.0
        exp = snap.meter_export.value or 0.0
        snap.meter_balance = Reading(
            "Balance en vatímetro",
            value=exp - imp,
            unit="kW",
        )

        prod_kw = (snap.inverter_power.value or 0) / 1000
        # Site consumption = production - net export at metering point
        snap.site_consumption = Reading(
            "Consumo instalación",
            value=prod_kw - (exp - imp),
            unit="kW",
        )

        if "Potencia aparente total (TotVA)" in meter:
            snap.meter_apparent_total = meter["Potencia aparente total (TotVA)"]
        if "Frecuencia" in meter:
            snap.meter_frequency = meter["Frecuencia"]

        if probe_phases:
            for key in (
                "Exportación activa L1",
                "Exportación activa L2",
                "Exportación activa L3",
            ):
                if key in meter and meter[key].value is not None:
                    snap.meter_export_phases.append(meter[key])
            for key in (
                "Potencia aparente L1",
                "Potencia aparente L2",
                "Potencia aparente L3",
            ):
                if key in meter and meter[key].value is not None:
                    snap.meter_apparent_phases.append(meter[key])
            for key in (
                "Tensión fase L1",
                "Tensión fase L2",
                "Tensión fase L3",
            ):
                if key in meter and meter[key].value is not None:
                    snap.meter_voltage.append(meter[key])

        for label, reading in inv.items():
            if label != "Potencia AC" and (reading.value is not None or probe_strings):
                snap.dc_strings.append(reading)

        if self.stale_check:
            stale_reason = self._check_meter_stale(
                inv_raw_start,
                inv_raw_end,
                meter_fp_start,
                meter_fp_end,
                snap.inverter_power.value,
                snap.meter_export.value,
                snap.meter_import.value,
            )
            if stale_reason:
                invalidate_meter_snapshot(snap, stale_reason)

        return snap


def invalidate_meter_snapshot(snap: PlantSnapshot, reason: str) -> None:
    snap.meter_stale = True
    snap.meter_stale_reason = reason
    for reading in (
        snap.meter_import,
        snap.meter_export,
        snap.meter_balance,
        snap.site_consumption,
        snap.meter_apparent_total,
        snap.meter_frequency,
    ):
        reading.value = None
        reading.error = reason
    snap.meter_export_phases = [
        Reading(r.label, unit=r.unit, error=reason) for r in snap.meter_export_phases
    ]
    snap.meter_apparent_phases = [
        Reading(r.label, unit=r.unit, error=reason) for r in snap.meter_apparent_phases
    ]
    snap.meter_voltage = [Reading(r.label, unit=r.unit, error=reason) for r in snap.meter_voltage]


def reading_to_dict(reading: Reading) -> dict:
    return {
        "label": reading.label,
        "value": reading.value,
        "unit": reading.unit,
        "error": reading.error,
    }


def snapshot_to_dict(snap: PlantSnapshot, elapsed_s: float | None = None) -> dict:
    data = {
        "timestamp": snap.timestamp,
        "host": snap.host,
        "inverter_unit": snap.inverter_unit,
        "meter_unit": snap.meter_unit,
        "inverter_power": reading_to_dict(snap.inverter_power),
        "meter_import": reading_to_dict(snap.meter_import),
        "meter_export": reading_to_dict(snap.meter_export),
        "meter_balance": reading_to_dict(snap.meter_balance),
        "site_consumption": reading_to_dict(snap.site_consumption),
        "meter_apparent_total": reading_to_dict(snap.meter_apparent_total),
        "meter_frequency": reading_to_dict(snap.meter_frequency),
        "meter_export_phases": [reading_to_dict(r) for r in snap.meter_export_phases],
        "meter_apparent_phases": [reading_to_dict(r) for r in snap.meter_apparent_phases],
        "meter_voltage": [reading_to_dict(r) for r in snap.meter_voltage],
        "dc_strings": [reading_to_dict(r) for r in snap.dc_strings],
        "meter_stale": snap.meter_stale,
        "meter_stale_reason": snap.meter_stale_reason,
    }
    if elapsed_s is not None:
        data["read_duration_s"] = round(elapsed_s, 2)
    return data


def print_snapshot(snap: PlantSnapshot, elapsed_s: float | None = None) -> None:
    print(f"\n{'='*60}")
    print(f"  SMA Plant — {snap.timestamp}")
    print(
        f"  Data Manager: {snap.host} | Inverter unit {snap.inverter_unit} | "
        f"Power meter unit {snap.meter_unit}"
    )
    print(f"{'='*60}")
    if elapsed_s is not None:
        print(f"  Modbus read time: {elapsed_s:.1f} s")

    print("\n--- Production ---")
    print(f"  {snap.inverter_power.label:30s} {snap.inverter_power.formatted()}")

    print("\n--- Power meter ---")
    if snap.meter_stale:
        print(f"  ⚠ {snap.meter_stale_reason}")
    print(f"  {snap.meter_import.label:30s} {snap.meter_import.formatted()}")
    print(f"  {snap.meter_export.label:30s} {snap.meter_export.formatted()}")
    print(f"  {snap.meter_balance.label:30s} {snap.meter_balance.formatted()}")
    print(f"  {snap.site_consumption.label:30s} {snap.site_consumption.formatted()}")
    print(f"  {snap.meter_apparent_total.label:30s} {snap.meter_apparent_total.formatted()}")
    print(f"  {snap.meter_frequency.label:30s} {snap.meter_frequency.formatted()}")
    print("\n  (Totals 30865/30867 are the reference for consumption and balance.)")

    if snap.meter_export_phases:
        phase_sum = sum(r.value or 0 for r in snap.meter_export_phases)
        print("\n--- Per-phase detail (--phases) ---")
        print("  Active export per phase (sum ≈ total exported):")
        for r in snap.meter_export_phases:
            print(f"    {r.label:28s} {r.formatted()}")
        print(f"    {'Phase sum':28s} {phase_sum:.2f} kW")
        if snap.meter_export_phases and snap.meter_apparent_phases:
            print("  Apparent power per phase (VA, not active export):")
            for r in snap.meter_apparent_phases:
                print(f"    {r.label:28s} {r.formatted()}")
        if snap.meter_voltage:
            print("  Phase voltages at power meter:")
            for r in snap.meter_voltage:
                print(f"    {r.label:28s} {r.formatted()}")

    print("\n--- Inverter strings / DC ---")
    if snap.dc_strings:
        for r in snap.dc_strings:
            print(f"  {r.label:30s} {r.formatted()}")
    else:
        print("  Not read (use --try-strings to try; usually fails on this model).")


DebugProbe = tuple[int, int, str, float, str, str]  # unit, addr, dtype, scale, unit_label, label


def _format_raw_regs(regs: list[int]) -> str:
    return f"[{', '.join(f'0x{r:04X} ({r})' for r in regs)}]"


def _scaled_value(raw: int | None, scale: float) -> str:
    if raw is None:
        return "N/A"
    return f"{raw * scale:.4f}"


def _debug_probes_for(reader: SmaPlantReader) -> list[DebugProbe]:
    return [
        (reader.inverter_unit, 30775, "s32", 1, "W", "Inversor Potencia AC (30775)"),
        (reader.meter_unit, 30865, "s32", 1 / 1000, "kW", "Vatímetro Import TotWIn (30865)"),
        (reader.meter_unit, 30867, "s32", 1 / 1000, "kW", "Vatímetro Export TotWOut (30867)"),
        (reader.meter_unit, 31447, "u32", 0.01, "Hz", "Vatímetro Frecuencia (31447)"),
        (reader.meter_unit, 31455, "s32", 1 / 1000, "kVA", "Vatímetro TotVA (31455)"),
        (reader.meter_unit, 31259, "u32", 1 / 1000, "kW", "Vatímetro Export L1 (31259)"),
        (reader.meter_unit, 31261, "u32", 1 / 1000, "kW", "Vatímetro Export L2 (31261)"),
        (reader.meter_unit, 31263, "u32", 1 / 1000, "kW", "Vatímetro Export L3 (31263)"),
    ]


def debug_modbus(
    reader: SmaPlantReader,
    samples: int = 5,
    pause_s: float = 2.0,
) -> None:
    """Repeated reads with raw registers to detect frozen values."""
    probes = _debug_probes_for(reader)
    print(f"Modbus debug — {reader.host}:{reader.port}")
    print(
        f"Inverter unit={reader.inverter_unit} | Power meter unit={reader.meter_unit} | "
        f"{samples} samples every {pause_s}s\n"
    )

    client = ModbusTcpClient(reader.host, port=reader.port, timeout=reader.timeout, retries=0)
    if not client.connect():
        print("ERROR: no connection to Data Manager", file=sys.stderr)
        return

    history: dict[str, list[str]] = {label: [] for *_, label in probes}

    try:
        for sample in range(1, samples + 1):
            ts = time.strftime("%H:%M:%S")
            print(f"--- Sample {sample}/{samples} [{ts}] ---")

            for unit_id, addr, dtype, scale, unit_label, label in probes:
                try:
                    response = client.read_holding_registers(
                        address=addr, count=2, device_id=unit_id
                    )
                    if response.isError():
                        line = f"ERROR {response}"
                    else:
                        raw = decode(response.registers, dtype)
                        line = (
                            f"raw={_scaled_value(raw, scale)} {unit_label} "
                            f"({_format_raw_regs(response.registers)})"
                        )
                except ModbusException as exc:
                    line = f"ERROR Modbus: {exc}"
                except Exception as exc:  # noqa: BLE001
                    line = f"ERROR: {exc}"

                history[label].append(line)
                print(f"  {label:<40s} {line}")

            # Compare block read vs individual for import/export
            try:
                block = client.read_holding_registers(
                    address=30865, count=4, device_id=reader.meter_unit
                )
                if block.isError():
                    print(f"  Block 30865×4                      ERROR {block}")
                else:
                    imp = decode(block.registers[0:2], "s32")
                    exp = decode(block.registers[2:4], "s32")
                    print(
                        f"  Block 30865×4                      "
                        f"import={_scaled_value(imp, 1/1000)} kW | "
                        f"export={_scaled_value(exp, 1/1000)} kW | "
                        f"{_format_raw_regs(block.registers)}"
                    )
            except Exception as exc:  # noqa: BLE001
                print(f"  Block 30865×4                      ERROR: {exc}")

            if sample < samples:
                time.sleep(pause_s)
            print()

        print("--- Summary: does it change between samples? ---")
        for label, values in history.items():
            unique = len(set(values))
            status = "VARIES" if unique > 1 else "FROZEN"
            print(f"  {label:<40s} {status} ({unique} distinct value(s))")
    finally:
        client.close()


def watch(reader: SmaPlantReader, interval: float) -> None:
    print(f"Monitoring every {interval}s (Ctrl+C to exit)\n")
    try:
        while True:
            snap = reader.read_snapshot()
            prod = snap.inverter_power.formatted()
            cons = snap.site_consumption.formatted()
            bal = snap.meter_balance.formatted()
            exp = snap.meter_export.formatted()
            print(
                f"[{snap.timestamp}]  "
                f"Production={prod} | Site consumption={cons} | "
                f"Export={exp} | Balance={bal}"
            )
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Lee planta SMA: inversor + vatímetro vía Data Manager")
    parser.add_argument("-a", "--address", default=DEFAULT_HOST, help="IP del Data Manager")
    parser.add_argument("-p", "--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--inverter-unit", type=int, default=INVERTER_UNIT)
    parser.add_argument("--meter-unit", type=int, default=METER_UNIT)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--delay", type=float, default=None, help="Pausa entre lecturas (s)")
    parser.add_argument(
        "--slow",
        action="store_true",
        help="Modo red lenta: timeout 20s, pausa 1s, más reintentos",
    )
    parser.add_argument("-w", "--watch", action="store_true")
    parser.add_argument("-i", "--interval", type=float, default=10.0, help="Intervalo monitor (s)")
    parser.add_argument(
        "--try-strings",
        action="store_true",
        help="Intentar leer strings/DC del inversor (suele fallar en este modelo)",
    )
    parser.add_argument(
        "--phases",
        action="store_true",
        help="Mostrar detalle por fase del vatímetro (exportación activa, VA, tensión)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Depuración: lecturas raw repetidas para detectar registros congelados",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Número de muestras con --debug (default: 5)",
    )
    parser.add_argument(
        "--debug-interval",
        type=float,
        default=2.0,
        help="Segundos entre muestras con --debug (default: 2)",
    )
    parser.add_argument(
        "--no-stale-check",
        action="store_true",
        help="No invalidar lecturas del vatímetro aunque parezcan obsoletas",
    )
    args = parser.parse_args()

    timeout = SLOW_TIMEOUT if args.slow else args.timeout
    delay = SLOW_DELAY if args.slow else (args.delay if args.delay is not None else DEFAULT_DELAY)
    retries = SLOW_RETRIES if args.slow else DEFAULT_RETRIES

    reader = SmaPlantReader(
        host=args.address,
        port=args.port,
        inverter_unit=args.inverter_unit,
        meter_unit=args.meter_unit,
        timeout=timeout,
        delay=delay,
        retries=retries,
    )
    reader.try_strings = args.try_strings
    reader.show_phases = args.phases
    reader.stale_check = not args.no_stale_check

    if args.debug:
        debug_modbus(reader, samples=args.samples, pause_s=args.debug_interval)
    elif args.watch:
        watch(reader, args.interval)
    else:
        t0 = time.monotonic()
        snap = reader.read_snapshot(
            try_strings=args.try_strings, show_phases=args.phases
        )
        print_snapshot(snap, elapsed_s=time.monotonic() - t0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())