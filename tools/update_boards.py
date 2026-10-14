#!/usr/bin/env python3
"""Refresh boards/boards.json from PlatformIO's platform-espressif32 board list.

Sources, tried in order (or pick one with --source):
  local   an installed PlatformIO platform (~/.platformio/platforms/espressif32/boards)
  git     shallow sparse clone of github.com/platformio/platform-espressif32
  api     GitHub contents API + raw.githubusercontent.com downloads

Only the Python standard library (and `git` for the git source) is required.
"""
import argparse
import datetime
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import urllib.request

REPO = "platformio/platform-espressif32"
BRANCH = "develop"
HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_OUT = HERE.parent / "boards" / "boards.json"


def summarize(board_id, manifest):
    build = manifest.get("build", {})
    upload = manifest.get("upload", {})
    f_cpu = str(build.get("f_cpu", "0")).rstrip("Ll")
    return {
        "id": board_id,
        "name": manifest.get("name", board_id),
        "mcu": build.get("mcu", ""),
        "f_cpu": int(f_cpu) if f_cpu.isdigit() else 0,
        "flash_size": upload.get("flash_size", ""),
        "ram": upload.get("maximum_ram_size", 0),
        "vendor": manifest.get("vendor", ""),
        "url": manifest.get("url", ""),
    }


def from_dir(path):
    boards = []
    for f in sorted(pathlib.Path(path).glob("*.json")):
        with open(f, encoding="utf-8") as fh:
            boards.append(summarize(f.stem, json.load(fh)))
    return boards


def source_local():
    core = os.environ.get("PLATFORMIO_CORE_DIR", os.path.expanduser("~/.platformio"))
    path = pathlib.Path(core) / "platforms" / "espressif32" / "boards"
    if not path.is_dir():
        raise RuntimeError(f"{path} not found (is the espressif32 platform installed?)")
    return from_dir(path), f"local PlatformIO install {path}"


def source_git():
    if not shutil.which("git"):
        raise RuntimeError("git not found")
    with tempfile.TemporaryDirectory() as tmp:
        url = f"https://github.com/{REPO}.git"
        run = lambda *a: subprocess.run(a, cwd=tmp, check=True, capture_output=True)
        run("git", "clone", "--depth", "1", "--filter=blob:none", "--sparse", "--branch", BRANCH, url, "repo")
        repo = pathlib.Path(tmp) / "repo"
        subprocess.run(["git", "sparse-checkout", "set", "boards"], cwd=repo, check=True, capture_output=True)
        rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=repo, check=True,
                             capture_output=True, text=True).stdout.strip()
        return from_dir(repo / "boards"), f"github.com/{REPO}@{rev}"


def fetch_json(url):
    req = urllib.request.Request(url, headers={"User-Agent": "esp32sim-update-boards"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)


def source_api():
    listing = fetch_json(f"https://api.github.com/repos/{REPO}/contents/boards?ref={BRANCH}")
    boards = []
    for entry in listing:
        if entry["name"].endswith(".json"):
            boards.append(summarize(entry["name"][:-5], fetch_json(entry["download_url"])))
    return boards, f"github.com/{REPO} ({BRANCH}, via API)"


SOURCES = {"local": source_local, "git": source_git, "api": source_api}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=SOURCES, help="use only this source")
    ap.add_argument("-o", "--output", default=str(DEFAULT_OUT))
    args = ap.parse_args()

    errors = []
    for name in ([args.source] if args.source else ["local", "git", "api"]):
        try:
            boards, origin = SOURCES[name]()
            break
        except Exception as e:  # noqa: BLE001 - report and try the next source
            errors.append(f"{name}: {e}")
    else:
        sys.exit("could not fetch the board list:\n  " + "\n  ".join(errors))

    boards.sort(key=lambda b: b["id"])
    out = {
        "source": origin,
        "generated": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d"),
        "boards": boards,
    }
    with open(args.output, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    by_mcu = {}
    for b in boards:
        by_mcu[b["mcu"]] = by_mcu.get(b["mcu"], 0) + 1
    print(f"wrote {len(boards)} boards from {origin} to {args.output}")
    print("  " + ", ".join(f"{k}: {v}" for k, v in sorted(by_mcu.items())))


if __name__ == "__main__":
    main()
