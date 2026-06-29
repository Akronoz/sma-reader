#!/usr/bin/env python3
"""
Lector Modbus TCP para instalación SMA con Data Manager.

Configuración verificada en red local:
  - Data Manager: 192.168.1.101:502
  - Inversor 100 kW: unit_id=10
  - Vatímetro:       unit_id=11

Uso:
  python sma_plant.py                  # lectura rápida (~3-10 s)
  python sma_plant.py --slow           # si la WiFi del inversor es inestable
  python sma_plant.py -w -i 10         # monitor cada 10 s
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# --- Configuración por defecto ---
DEFAULT_HOST = "192.168.1.101"
DEFAULT_PORT = 502
INVERTER_UNIT = 10
METER_UNIT = 11

# Por defecto: lecturas en bloque, pocas peticiones (típ. 2-8 s en LAN/WiFi normal).
# Si falla por red lenta, usa: --slow
DEFAULT_TIMEOUT = 5.0
DEFAULT_DELAY = 0.15
DEFAULT_RETRIES = 1
SLOW_TIMEOUT = 20.0
SLOW_DELAY = 1.0
SLOW_RETRIES = 3

NAN_S32 = 0x80000000
NAN_U32 = 0xFFFFFFFF

RegisterDef = tuple[int, str, float, str, str]  # addr, dtype, scale, unit, label
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


# Inversor (unit 10): potencia AC verificada
INVERTER_POWER: RegisterDef = (30775, "s32", 1, "W", "Potencia AC")

# Strings/DC: no disponibles en este inversor 100 kW vía Data Manager (error Modbus 4).
# Se intentan solo con --try-strings (1 reintento, sin alargar la lectura habitual).
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

# Vatímetro (unit 11): usar totales para consumo/balance (registros fiables).
METER_CORE_REGISTERS: list[RegisterDef] = [
    (30865, "s32", 1 / 1000, "kW", "Importación red (TotWIn)"),
    (30867, "s32", 1 / 1000, "kW", "Exportación red (TotWOut)"),
    (31455, "s32", 1 / 1000, "kVA", "Potencia aparente total (TotVA)"),
    (31447, "u32", 0.01, "Hz", "Frecuencia"),
]

# Detalle por fase (opcional con --phases). La suma de exportación L1+L2+L3 = TotWOut.
# Las corrientes 31435×V ≈ VA de fase, NO la potencia activa exportada por fase.
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

    def _read_with_retries(
        self,
        client: ModbusTcpClient,
        unit_id: int,
        addr: int,
        dtype: str,
        retries: int,
    ) -> tuple[int | None, str | None]:
        last_err = "Error desconocido"
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
                results[label] = Reading(label, unit=unit, error="Sin conexión")
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
        last_err = "Error desconocido"
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
                results[label] = Reading(label, unit=unit, error="Fuera de bloque")
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
    ) -> tuple[dict[str, Reading], dict[str, Reading]]:
        inv: dict[str, Reading] = {}
        meter: dict[str, Reading] = {}

        client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout, retries=0)
        if not client.connect():
            inv["Potencia AC"] = Reading("Potencia AC", unit="W", error="Sin conexión")
            for _, _, _, unit, label in METER_CORE_REGISTERS:
                meter[label] = Reading(label, unit=unit, error="Sin conexión")
            return inv, meter

        try:
            # Inversor: 1 petición
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

            # Vatímetro: 3 peticiones (TotWIn+TotWOut, Hz, TotVA)
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

        return inv, meter

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

        inv, meter = self._read_snapshot_fast(probe_strings, probe_phases)

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
        # Consumo instalación = producción - exportación neta al punto de medida
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

        return snap


def print_snapshot(snap: PlantSnapshot, elapsed_s: float | None = None) -> None:
    print(f"\n{'='*60}")
    print(f"  Instalación SMA — {snap.timestamp}")
    print(
        f"  Data Manager: {snap.host} | Inversor unit {snap.inverter_unit} | "
        f"Vatímetro unit {snap.meter_unit}"
    )
    print(f"{'='*60}")
    if elapsed_s is not None:
        print(f"  Tiempo de lectura Modbus: {elapsed_s:.1f} s")

    print("\n--- Producción ---")
    print(f"  {snap.inverter_power.label:30s} {snap.inverter_power.formatted()}")

    print("\n--- Vatímetro ---")
    print(f"  {snap.meter_import.label:30s} {snap.meter_import.formatted()}")
    print(f"  {snap.meter_export.label:30s} {snap.meter_export.formatted()}")
    print(f"  {snap.meter_balance.label:30s} {snap.meter_balance.formatted()}")
    print(f"  {snap.site_consumption.label:30s} {snap.site_consumption.formatted()}")
    print(f"  {snap.meter_apparent_total.label:30s} {snap.meter_apparent_total.formatted()}")
    print(f"  {snap.meter_frequency.label:30s} {snap.meter_frequency.formatted()}")
    print("\n  (Totales 30865/30867 son la referencia para consumo y balance.)")

    if snap.meter_export_phases:
        phase_sum = sum(r.value or 0 for r in snap.meter_export_phases)
        print("\n--- Detalle por fase (--phases) ---")
        print("  Exportación activa por fase (suma ≈ total exportado):")
        for r in snap.meter_export_phases:
            print(f"    {r.label:28s} {r.formatted()}")
        print(f"    {'Suma fases':28s} {phase_sum:.2f} kW")
        if snap.meter_export_phases and snap.meter_apparent_phases:
            print("  Potencia aparente por fase (VA, no es exportación activa):")
            for r in snap.meter_apparent_phases:
                print(f"    {r.label:28s} {r.formatted()}")
        if snap.meter_voltage:
            print("  Tensiones de fase en el vatímetro:")
            for r in snap.meter_voltage:
                print(f"    {r.label:28s} {r.formatted()}")

    print("\n--- Strings / DC inversor ---")
    if snap.dc_strings:
        for r in snap.dc_strings:
            print(f"  {r.label:30s} {r.formatted()}")
    else:
        print("  No leídos (usa --try-strings para intentar; suele fallar en este modelo).")


def watch(reader: SmaPlantReader, interval: float) -> None:
    print(f"Monitorizando cada {interval}s (Ctrl+C para salir)\n")
    try:
        while True:
            snap = reader.read_snapshot()
            prod = snap.inverter_power.formatted()
            cons = snap.site_consumption.formatted()
            bal = snap.meter_balance.formatted()
            exp = snap.meter_export.formatted()
            print(
                f"[{snap.timestamp}]  "
                f"Producción={prod} | Consumo instalación={cons} | "
                f"Exportación={exp} | Balance={bal}"
            )
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nMonitor detenido.")


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

    if args.watch:
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