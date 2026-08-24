#!/usr/bin/env python3
"""Upload a compiled .hex to the Arduino Uno over its bootloader.

The Uno is wired to the Raspberry Pi, not to the workstation, so reflashing it
otherwise means unplugging the robot and carrying it to a machine with the
Arduino IDE. This does it in place, over the same serial port the base
controller talks on, with nothing installed beyond pyserial:

    python3 scripts/flash_arduino.py build/base.hex
    python3 scripts/flash_arduino.py build/base.hex --port /dev/arduino_mini

It speaks STK500v1, which is what Optiboot (the Uno's stock bootloader) speaks,
and it is deliberately narrow: read the signature, write the pages, read them
back, leave. It cannot touch the bootloader itself -- writes above the
application section are refused -- so a failed upload leaves a board that still
takes uploads.

Getting a .hex to upload: the Arduino IDE writes one into its build directory,
or use scripts/build_arduino.sh if you have the IDE's toolchain installed.

Stop anything using the port first (the ROS serial bridge holds it open).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:  # pragma: no cover - runtime dependency check
    sys.exit("pyserial is required: sudo apt install python3-serial")

# STK500v1, the subset Optiboot implements.
STK_OK = 0x10
STK_INSYNC = 0x14
CRC_EOP = 0x20
STK_GET_SYNC = 0x30
STK_ENTER_PROGMODE = 0x50
STK_LEAVE_PROGMODE = 0x51
STK_LOAD_ADDRESS = 0x55
STK_PROG_PAGE = 0x64
STK_READ_PAGE = 0x74
STK_READ_SIGN = 0x75

# ATmega328P as fitted to the Uno.
SIGNATURE = (0x1E, 0x95, 0x0F)
PAGE_SIZE = 128
# The bootloader lives in the top 512 bytes. Nothing this script writes may
# reach it: overwriting the bootloader is the one mistake that turns a board
# into something only an ISP programmer can recover.
APPLICATION_LIMIT = 32256

BOOTLOADER_BAUD = 115200


def read_intel_hex(path: Path) -> bytearray:
    """Flatten an Intel HEX file into a flat image starting at address 0."""
    image = bytearray()
    extended_base = 0

    for number, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(':'):
            raise ValueError(f"{path}:{number}: not an Intel HEX record")

        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError(f"{path}:{number}: truncated record")
        if (sum(record) & 0xFF) != 0:
            raise ValueError(f"{path}:{number}: checksum mismatch")

        count, address, record_type = record[0], (record[1] << 8) | record[2], record[3]
        data = record[4:4 + count]

        if record_type == 0x00:            # data
            start = extended_base + address
            if len(image) < start + count:
                image.extend(b'\xff' * (start + count - len(image)))
            image[start:start + count] = data
        elif record_type == 0x01:          # end of file
            break
        elif record_type == 0x02:          # extended segment address
            extended_base = ((data[0] << 8) | data[1]) * 16
        elif record_type == 0x04:          # extended linear address
            extended_base = ((data[0] << 8) | data[1]) << 16
        # 0x03/0x05 are start-address records: meaningless on AVR, ignored.

    if not image:
        raise ValueError(f"{path}: no program data")
    return image


class Programmer:
    """One bootloader session: sync, program, verify, leave."""

    def __init__(self, port: str, timeout: float = 1.0) -> None:
        self.serial = serial.Serial(port, BOOTLOADER_BAUD, timeout=timeout)

    def close(self) -> None:
        self.serial.close()

    def reset_board(self) -> None:
        # The Uno resets when DTR falls; the bootloader then listens for about
        # a second before handing over to the sketch.
        for line in ('dtr', 'rts'):
            setattr(self.serial, line, False)
        time.sleep(0.12)
        for line in ('dtr', 'rts'):
            setattr(self.serial, line, True)
        time.sleep(0.12)
        for line in ('dtr', 'rts'):
            setattr(self.serial, line, False)
        time.sleep(0.20)
        self.serial.reset_input_buffer()

    def command(self, payload: bytes, response_length: int = 0) -> bytes:
        """Send one command and return its response body.

        Every STK500v1 reply is INSYNC, the body, then OK. Anything else means
        the board is not in its bootloader -- most often because the sketch is
        already running, or something else holds the port.
        """
        self.serial.write(payload + bytes([CRC_EOP]))
        self.serial.flush()

        head = self.serial.read(1)
        if head != bytes([STK_INSYNC]):
            raise IOError(f"expected INSYNC, got {head.hex() or '<timeout>'}")
        body = self.serial.read(response_length) if response_length else b''
        if len(body) != response_length:
            raise IOError("short reply from the bootloader")
        tail = self.serial.read(1)
        if tail != bytes([STK_OK]):
            raise IOError(f"expected OK, got {tail.hex() or '<timeout>'}")
        return body

    def sync(self, attempts: int = 6) -> None:
        last = None
        for attempt in range(attempts):
            try:
                self.reset_board()
                self.command(bytes([STK_GET_SYNC]))
                return
            except IOError as exc:
                last = exc
                time.sleep(0.15 * (attempt + 1))
        raise IOError(f"cannot reach the bootloader: {last}")

    def check_signature(self) -> tuple[int, ...]:
        signature = tuple(self.command(bytes([STK_READ_SIGN]), 3))
        if signature != SIGNATURE:
            raise IOError(
                f"unexpected device signature {['0x%02X' % b for b in signature]}; "
                f"expected an ATmega328P {['0x%02X' % b for b in SIGNATURE]}")
        return signature

    def load_address(self, byte_address: int) -> None:
        # STK500 addresses flash in words, not bytes.
        word = byte_address >> 1
        self.command(bytes([STK_LOAD_ADDRESS, word & 0xFF, (word >> 8) & 0xFF]))

    def write_page(self, byte_address: int, data: bytes) -> None:
        self.load_address(byte_address)
        header = bytes([STK_PROG_PAGE, (len(data) >> 8) & 0xFF, len(data) & 0xFF, ord('F')])
        self.command(header + data)

    def read_page(self, byte_address: int, length: int) -> bytes:
        self.load_address(byte_address)
        header = bytes([STK_READ_PAGE, (length >> 8) & 0xFF, length & 0xFF, ord('F')])
        return self.command(header, length)


def upload(path: Path, port: str, verify: bool = True) -> int:
    image = read_intel_hex(path)
    if len(image) > APPLICATION_LIMIT:
        sys.exit(f"{path} is {len(image)} bytes; the application section holds "
                 f"{APPLICATION_LIMIT}. Refusing to write into the bootloader.")

    # Pad to a whole page so the last write is not a partial one.
    if len(image) % PAGE_SIZE:
        image.extend(b'\xff' * (PAGE_SIZE - len(image) % PAGE_SIZE))

    print(f"{path.name}: {len(image)} bytes "
          f"({100 * len(image) / APPLICATION_LIMIT:.0f}% of the application section)")

    programmer = Programmer(port)
    try:
        programmer.sync()
        signature = programmer.check_signature()
        print(f"bootloader on {port}, signature "
              f"{' '.join('0x%02X' % b for b in signature)} (ATmega328P)")

        programmer.command(bytes([STK_ENTER_PROGMODE]))
        for offset in range(0, len(image), PAGE_SIZE):
            programmer.write_page(offset, bytes(image[offset:offset + PAGE_SIZE]))
        print(f"wrote {len(image) // PAGE_SIZE} pages")

        if verify:
            for offset in range(0, len(image), PAGE_SIZE):
                page = programmer.read_page(offset, PAGE_SIZE)
                if page != bytes(image[offset:offset + PAGE_SIZE]):
                    sys.exit(f"VERIFY FAILED at 0x{offset:04X}. The board is in an "
                             f"unknown state -- re-run before powering it down.")
            print("verified")

        programmer.command(bytes([STK_LEAVE_PROGMODE]))
    finally:
        programmer.close()

    print("done; the board is running the new sketch")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("hex_file", type=Path, help="compiled .hex to upload")
    parser.add_argument("--port", default="/dev/arduino_mini",
                        help="serial port the Uno is on (default: %(default)s)")
    parser.add_argument("--no-verify", action="store_true",
                        help="skip the read-back check (not recommended)")
    args = parser.parse_args()

    if not args.hex_file.is_file():
        sys.exit(f"no such file: {args.hex_file}")
    return upload(args.hex_file, args.port, verify=not args.no_verify)


if __name__ == "__main__":
    raise SystemExit(main())
