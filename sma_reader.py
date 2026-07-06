#!/usr/bin/env python3
"""Modbus TCP reader for SMA inverters."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass
from typing import Callable

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

RegisterDef = tuple[int, int, str, float, str, str]

# Registers verified on the local network 100 kW inverter.
CORE_REGISTERS: list[RegisterDef] = [
    (30775, 2, "s32", 1, "GridMs.TotW", "Potencia AC total"),
    (30513, 4, "u64", 1, "Metering.TotWhOut", "Energía total producida"),
    (30517, 4, "u64", 1, "Metering.DyWhOut", "Energía del día"),
    (30201, 2, "u32", 1, "Operation.Health", "Estado del inversor"),
    (30005, 2, "u32", 1, "Nameplate.SerNum", "Número de serie"),
]

# Additional registers per SMA documentation. May not be available
# on all models or gateway configurations.
EXTENDED_REGISTERS: list[RegisterDef] = [
    (30777, 2, "s32", 1, "GridMs.W.phsA", "Potencia L1"),
    (30779, 2, "s32", 1, "GridMs.W.phsB", "Potencia L2"),
    (30781, 2, "s32", 1, "GridMs.W.phsC", "Potencia L3"),
    (30783, 2, "u32", 0.01, "GridMs.PhV.phsA", "Tensión L1"),
    (30785, 2, "u32", 0.01, "GridMs.PhV.phsB", "Tensión L2"),
    (30787, 2, "u32", 0.01, "GridMs.PhV.phsC", "Tensión L3"),
    (30803, 2, "u32", 0.01, "GridMs.Hz", "Frecuencia red"),
    (30795, 2, "u32", 0.001, "GridMs.TotA", "Corriente AC total"),
    (30977, 2, "s32", 0.001, "GridMs.A.phsA", "Corriente L1"),
    (30979, 2, "s32", 0.001, "GridMs.A.phsB", "Corriente L2"),
    (30981, 2, "s32", 0.001, "GridMs.A.phsC", "Corriente L3"),
    (30769, 2, "s32", 0.001, "DcMs.Amp.MPPT1", "Corriente DC MPPT1"),
    (30771, 2, "s32", 0.01, "DcMs.Vol.MPPT1", "Tensión DC MPPT1"),
    (30773, 2, "s32", 1, "DcMs.Watt.MPPT1", "Potencia DC MPPT1"),
    (30957, 2, "s32", 0.001, "DcMs.Amp.MPPT2", "Corriente DC MPPT2"),
    (30959, 2, "s32", 0.01, "DcMs.Vol.MPPT2", "Tensión DC MPPT2"),
    (30961, 2, "s32", 1, "DcMs.Watt.MPPT2", "Potencia DC MPPT2"),
    (30953, 2, "s32", 0.1, "Coolsys.Cab.TmpVal", "Temperatura interior"),
]

REGISTERS = CORE_REGISTERS + EXTENDED_REGISTERS

HEALTH_STATUS = {
    303: "Off",
    307: "Ok",
    455: "Warning",
    1392: "Error",
    308: "On",
}

NAN_VALUES = {
    "s16": -32768,
    "s32": -2147483648,
    "u32": 0xFFFFFFFF,
    "u64": 0xFFFFFFFFFFFFFFFF,
}

UNITS = {
    "GridMs.TotW": "W",
    "GridMs.W.phsA": "W",
    "GridMs.W.phsB": "W",
    "GridMs.W.phsC": "W",
    "GridMs.PhV.phsA": "V",
    "GridMs.PhV.phsB": "V",
    "GridMs.PhV.phsC": "V",
    "GridMs.Hz": "Hz",
    "GridMs.TotA": "A",
    "GridMs.A.phsA": "A",
    "GridMs.A.phsB": "A",
    "GridMs.A.phsC": "A",
    "DcMs.Amp.MPPT1": "A",
    "DcMs.Vol.MPPT1": "V",
    "DcMs.Watt.MPPT1": "W",
    "DcMs.Amp.MPPT2": "A",
    "DcMs.Vol.MPPT2": "V",
    "DcMs.Watt.MPPT2": "W",
    "Metering.TotWhOut": "Wh",
    "Metering.DyWhOut": "Wh",
    "Coolsys.Cab.TmpVal": "°C",
}


@dataclass
class RegisterValue:
    address: int
    name: str
    description: str
    value: float | int | str | None
    unit: str
    raw_registers: list[int] | None = None
    error: str | None = None


def decode_registers(registers: list[int], dtype: str) -> int:
    if dtype == "s16":
        value = registers[0]
        return value - 65536 if value > 32767 else value
    if dtype == "u32":
        return (registers[0] << 16) | registers[1]
    if dtype == "s32":
        value = (registers[0] << 16) | registers[1]
        return value - 4294967296 if value > 2147483647 else value
    if dtype == "u64":
        return (
            (registers[0] << 48)
            | (registers[1] << 32)
            | (registers[2] << 16)
            | registers[3]
        )
    return registers[0]


def format_value(name: str, value: float | int, unit: str) -> str:
    if name == "Operation.Health":
        return HEALTH_STATUS.get(int(value), str(int(value)))

    if unit == "W" and abs(value) >= 1000:
        return f"{value / 1000:.2f} kW"
    if unit == "Wh" and abs(value) >= 1_000_000:
        return f"{value / 1_000_000:.2f} MWh"
    if unit == "Wh" and abs(value) >= 1000:
        return f"{value / 1000:.2f} kWh"
    if unit:
        return f"{value:,.3f} {unit}".rstrip("0").rstrip(".")
    if name == "Nameplate.SerNum":
        return str(int(value))
    return str(int(value)) if float(value).is_integer() else str(value)


class SmaModbusReader:
    def __init__(
        self,
        host: str,
        port: int = 502,
        unit_id: int = 1,
        timeout: float = 5.0,
        delay: float = 0.2,
        retries: int = 2,
    ) -> None:
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.delay = delay
        self.retries = retries
        self._client: ModbusTcpClient | None = None

    def connect(self) -> bool:
        self._client = ModbusTcpClient(
            self.host, port=self.port, timeout=self.timeout, retries=1
        )
        return self._client.connect()

    def close(self) -> None:
        if self._client:
            self._client.close()
            self._client = None

    def __enter__(self) -> "SmaModbusReader":
        if not self.connect():
            raise ConnectionError(f"Could not connect to {self.host}:{self.port}")
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def read_register(
        self,
        address: int,
        count: int,
        dtype: str,
        scale: float,
        name: str,
        description: str,
    ) -> RegisterValue:
        unit = UNITS.get(name, "")
        if not self._client:
            return RegisterValue(address, name, description, None, unit, error="No connection")

        last_error = "Unknown error"
        for attempt in range(self.retries + 1):
            try:
                response = self._client.read_holding_registers(
                    address=address,
                    count=count,
                    device_id=self.unit_id,
                )
                if response.isError():
                    last_error = str(response)
                    time.sleep(self.delay * (attempt + 1))
                    continue

                raw = decode_registers(response.registers, dtype)
                if raw == NAN_VALUES.get(dtype):
                    return RegisterValue(
                        address, name, description, None, unit, response.registers
                    )

                scaled = raw * scale
                if name == "Operation.Health":
                    return RegisterValue(
                        address,
                        name,
                        description,
                        HEALTH_STATUS.get(int(scaled), int(scaled)),
                        "",
                        response.registers,
                    )

                return RegisterValue(
                    address, name, description, scaled, unit, response.registers
                )
            except ModbusException as exc:
                last_error = str(exc)
            except Exception as exc:  # noqa: BLE001
                last_error = str(exc)

            time.sleep(self.delay * (attempt + 1))

        return RegisterValue(address, name, description, None, unit, error=last_error)

    def read_all(self) -> list[RegisterValue]:
        results: list[RegisterValue] = []
        for address, count, dtype, scale, name, description in REGISTERS:
            results.append(
                self.read_register(address, count, dtype, scale, name, description)
            )
            time.sleep(self.delay)
        return results


def discover_inverter(subnet: str | None = None, unit_ids: list[int] | None = None) -> str | None:
    """Search for an SMA inverter on the local network by scanning port 502."""
    if unit_ids is None:
        unit_ids = [1, 3]

    if subnet is None:
        local_ip = _get_local_ip()
        if not local_ip:
            return None
        subnet = ".".join(local_ip.split(".")[:3])

    print(f"Scanning {subnet}.0/24 on port 502...")

    for host in range(1, 255):
        ip = f"{subnet}.{host}"
        if not _port_open(ip, 502):
            continue

        print(f"  Port 502 open on {ip}, trying Modbus...")
        for unit_id in unit_ids:
            try:
                with SmaModbusReader(ip, unit_id=unit_id, delay=0.15) as reader:
                    result = reader.read_register(
                        30775, 2, "s32", 1, "GridMs.TotW", "Potencia AC total"
                    )
                    if result.value is not None or result.error is None:
                        health = reader.read_register(
                            30201, 2, "u32", 1, "Operation.Health", "Estado"
                        )
                        print(f"  -> SMA inverter detected: {ip} (unit_id={unit_id})")
                        if health.value:
                            print(f"     Status: {health.value}")
                        if result.value is not None:
                            print(
                                f"     Power: {format_value('GridMs.TotW', float(result.value), 'W')}"
                            )
                        return ip
            except ConnectionError:
                continue
            except Exception:
                continue

    return None


def _get_local_ip() -> str | None:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return None


def _port_open(host: str, port: int, timeout: float = 0.4) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def print_results(results: list[RegisterValue]) -> None:
    print(f"\n{'Register':<22} {'Description':<28} {'Value'}")
    print("-" * 72)
    for item in results:
        if item.error:
            value_str = f"ERROR: {item.error}"
        elif item.value is None:
            value_str = "N/A"
        elif isinstance(item.value, str):
            value_str = item.value
        else:
            value_str = format_value(item.name, float(item.value), item.unit)
        print(f"{item.name:<22} {item.description:<28} {value_str}")


def watch(reader: SmaModbusReader, interval: float, keys: list[str]) -> None:
    key_set = set(keys)
    register_map = {name: reg for reg in REGISTERS if (name := reg[4]) in key_set or not keys}

    print(f"Monitoring every {interval}s (Ctrl+C to exit)\n")
    try:
        while True:
            timestamp = time.strftime("%H:%M:%S")
            parts: list[str] = []
            for address, count, dtype, scale, name, description in (
                reg for reg in REGISTERS if reg[4] in register_map
            ):
                result = reader.read_register(
                    address, count, dtype, scale, name, description
                )
                if result.value is not None:
                    if isinstance(result.value, str):
                        parts.append(f"{description}={result.value}")
                    else:
                        parts.append(
                            f"{description}={format_value(name, float(result.value), result.unit)}"
                        )
                time.sleep(reader.delay)

            print(f"[{timestamp}]  {' | '.join(parts)}")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Lee valores en tiempo real de un inversor SMA vía Modbus TCP"
    )
    parser.add_argument(
        "-a",
        "--address",
        help="IP del inversor (si no se indica, se intenta descubrir en la red)",
    )
    parser.add_argument("-p", "--port", type=int, default=502, help="Puerto Modbus (default: 502)")
    parser.add_argument(
        "-u",
        "--unit",
        type=int,
        default=1,
        help="Unit ID Modbus (default: 1; algunos SMA usan 3)",
    )
    parser.add_argument(
        "-w",
        "--watch",
        action="store_true",
        help="Monitorizar valores en bucle",
    )
    parser.add_argument(
        "-i",
        "--interval",
        type=float,
        default=5.0,
        help="Intervalo en segundos para --watch (default: 5)",
    )
    parser.add_argument(
        "--discover",
        action="store_true",
        help="Buscar inversor SMA en la red local",
    )
    parser.add_argument(
        "--registers",
        nargs="*",
        default=[],
        help="Solo leer ciertos registros (ej: GridMs.TotW Metering.DyWhOut)",
    )
    parser.add_argument(
        "--extended",
        action="store_true",
        help="Incluir registros extendidos (tensión, DC, etc.)",
    )
    args = parser.parse_args()

    host = args.address
    if args.discover or not host:
        found = discover_inverter()
        if not found:
            print("No SMA inverter found on the network.", file=sys.stderr)
            print(
                "Verify that Modbus TCP is enabled in Sunny Explorer/portal.",
                file=sys.stderr,
            )
            return 1
        if not host:
            host = found
            print(f"\nUsing detected inverter: {host}\n")

    try:
        with SmaModbusReader(host, port=args.port, unit_id=args.unit) as reader:
            if args.watch:
                watch(reader, args.interval, args.registers)
            else:
                if args.registers:
                    reg_map = {r[4]: r for r in REGISTERS}
                    results = []
                    for name in args.registers:
                        if name not in reg_map:
                            print(f"Unknown register: {name}", file=sys.stderr)
                            continue
                        address, count, dtype, scale, reg_name, description = reg_map[name]
                        results.append(
                            reader.read_register(
                                address, count, dtype, scale, reg_name, description
                            )
                        )
                else:
                    selected = (
                        REGISTERS
                        if args.extended
                        else CORE_REGISTERS
                    )
                    results = []
                    for address, count, dtype, scale, name, description in selected:
                        results.append(
                            reader.read_register(
                                address, count, dtype, scale, name, description
                            )
                        )
                        time.sleep(reader.delay)

                print(f"SMA inverter: {host}:{args.port} (unit_id={args.unit})")
                print_results(results)
    except ConnectionError as exc:
        print(exc, file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())