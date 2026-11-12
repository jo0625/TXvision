#!/usr/bin/env python3
"""Build a standalone scope page with one recording selected by default."""
import argparse
import json
from pathlib import Path

from build_scope import build


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    record = json.loads(args.record.read_text())
    if not isinstance(record.get("rows", {}).get("feedback"), list):
        raise ValueError("record must contain rows.feedback")

    base = build(args.output).read_text()
    item = {
        "id": "imported-latest",
        "label": args.record.name,
        "filename": args.record.name,
        "data": record,
        "presets": {},
        "diagnosed_zero_targets": False,
    }
    marker = '{"records":['
    embedded = json.dumps(item, ensure_ascii=False, separators=(",", ":"))
    args.output.write_text(base.replace(marker, marker + embedded + ",", 1))
    print(args.output)


if __name__ == "__main__":
    main()
