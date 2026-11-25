"""Inspect a Portenta in QSPI service mode and download original BIN files."""
import argparse
from pathlib import Path

import serial

from binlog_20261122 import FILE_HEADER


MAX_BYTES = 16 * 1024 * 1024


def read_line(port):
    line = port.read_until(b"\n", 128)
    if not line.endswith(b"\n"):
        raise ValueError("No complete response from the Portenta QSPI service")
    return line.rstrip(b"\r\n")


def download(port, destination):
    header = read_line(port)
    if not header.startswith(b"BIN "):
        raise ValueError(header.decode("ascii", errors="replace"))
    count = int(header[4:])
    if not len(FILE_HEADER) <= count <= MAX_BYTES:
        raise ValueError("Invalid BIN length")
    if port.read(len(FILE_HEADER)) != FILE_HEADER:
        raise ValueError("Incomplete or unsupported PSILOG header")
    # Open exclusively and delete only a partial file created by this download.
    created = False
    try:
        with destination.open("xb") as file:
            created = True
            file.write(FILE_HEADER)
            remaining = count - len(FILE_HEADER)
            while remaining:
                chunk = port.read(min(remaining, 4096))
                if not chunk:
                    raise ValueError("Transfer timed out before the advertised length")
                file.write(chunk)
                remaining -= len(chunk)
            if read_line(port) != b"OK":
                raise ValueError("Missing transfer confirmation")
    except BaseException:
        if created:
            destination.unlink(missing_ok=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Portenta USB port, not the XBee adapter")
    commands = parser.add_subparsers(dest="action", required=True)
    commands.add_parser("info", help="read flash layout and ownership status")
    commands.add_parser("list", help="list archived BIN files")
    commands.add_parser("claim", help="authorize an empty, preformatted user volume for Avionics")
    get = commands.add_parser("get", help="download one BIN by session number")
    get.add_argument("session", type=int, choices=range(1000), metavar="0..999")
    get.add_argument("--out", type=Path)
    args = parser.parse_args()
    try:
        with serial.Serial(args.port, 115200, timeout=6, write_timeout=2) as port:
            if args.action == "get":
                destination = args.out or Path(f"qspi_{args.session:03d}.bin")
                if destination.exists():
                    raise FileExistsError(destination)
                port.write(f"GET {args.session:03d}\n".encode("ascii"))
                download(port, destination)
                print(destination)
            else:
                port.write((args.action.upper() + "\n").encode("ascii"))
                while True:
                    line = read_line(port).decode("ascii", errors="replace")
                    if line.startswith("ERR "):
                        raise ValueError(line)
                    if line == "OK" or line.startswith("OK "):
                        if line != "OK":
                            print(line)
                        break
                    print(line)
    except (OSError, ValueError) as error:
        parser.exit(1, f"QSPI service: {error}\n")


if __name__ == "__main__":
    main()
