#!/usr/bin/env python3
"""Download Espressif's mask-ROM ELF files into roms/.

  python3 tools/fetch_roms.py            # ESP32-C3 only
  python3 tools/fetch_roms.py --all      # every chip in the archive

The files come from https://github.com/espressif/esp-rom-elfs (Apache-2.0).
The archive is checked against a known SHA-256 before anything is extracted.
"""
import argparse
import hashlib
import io
import pathlib
import sys
import tarfile
import urllib.request

VERSION = "20241011"
URL = f"https://github.com/espressif/esp-rom-elfs/releases/download/{VERSION}/esp-rom-elfs-{VERSION}.tar.gz"
SHA256 = "921f000164a421c7628fbfee55b173384aafaa51883adc65cd27bf9b0af9e9a9"
DEFAULT = ["esp32c3_rev3_rom.elf"]

ROOT = pathlib.Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--all", action="store_true", help="extract the ROMs of every chip")
    ap.add_argument("--dest", default=str(ROOT / "roms"), help="output directory (default: roms/)")
    args = ap.parse_args()

    print(f"downloading {URL}")
    with urllib.request.urlopen(URL, timeout=60) as r:
        data = r.read()
    digest = hashlib.sha256(data).hexdigest()
    if digest != SHA256:
        sys.exit(f"checksum mismatch: got {digest}, expected {SHA256}")

    dest = pathlib.Path(args.dest)
    dest.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
        for m in tar.getmembers():
            name = pathlib.PurePosixPath(m.name).name
            if not m.isfile() or not name.endswith("_rom.elf"):
                continue
            if not args.all and name not in DEFAULT:
                continue
            (dest / name).write_bytes(tar.extractfile(m).read())
            print(f"  {dest / name}")


if __name__ == "__main__":
    main()
