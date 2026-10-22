#!/usr/bin/env python3
"""Cache the navigation table's NVIDIA YCB USDs and textures for offline use."""

import argparse
import json
import os
from pathlib import Path
import shutil
import urllib.request


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=Path(os.environ.get(
        "XLEROBOT_TABLETOP_ASSET_DIR", workspace / ".deps/isaac-props")))
    args = parser.parse_args()
    manifest = json.loads((workspace / "src/xlerobot_isaac/config/tabletop_objects.json").read_text())
    for prop in manifest["objects"]:
        name = prop["asset"]
        # The scene adds physics to the visual USD, which references its color map.
        # OmniPBR.mdl is supplied by Isaac Sim itself.
        for relative in (
            f"Axis_Aligned/{name}.usd",
            f"Axis_Aligned/Materials/Textures/{name}_COLOR.png",
        ):
            destination = args.destination / relative
            if destination.is_file() and destination.stat().st_size:
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_suffix(destination.suffix + ".part")
            print(f"Downloading {relative}", flush=True)
            try:
                with urllib.request.urlopen(manifest["asset_root"] + "/" + relative, timeout=60) as response:
                    with temporary.open("wb") as output:
                        shutil.copyfileobj(response, output)
                if not temporary.stat().st_size:
                    raise RuntimeError(f"Empty asset download: {relative}")
                temporary.replace(destination)
            finally:
                temporary.unlink(missing_ok=True)
    print(f"Tabletop assets ready: {args.destination.resolve()}")


if __name__ == "__main__":
    main()
