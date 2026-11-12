#!/usr/bin/env python3
"""Build a portable, offline history viewer with locally bundled libraries."""
import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_OUTPUT = ROOT / "frame_rate_diagnostics/gimbal_scope.html"


def build(output=DEFAULT_OUTPUT):
    sources = [
        ("deadband1_partial", "09-23 1度限制 · 断流中止",
         "dynamic_follow_deadband1_20260923_01.json", {}, False),
        ("deadband05_partial", "09-23 0.5度限制 · 断流中止",
         "dynamic_follow_deadband05_20260923_01.json", {}, False),
        ("steady_partial", "09-23 单调时钟修复后 · 提前结束",
         "dynamic_follow_steady_20260923_01.json", {}, False),
        ("incident", "09-23 异响记录", "dynamic_follow_clock_pulse_20260923_000348.json",
         {"post": [13, 17], "disabled": [19.5, 29.9]}, True),
        ("earlier", "09-22 动态跟随 · 3 分钟", "dynamic_follow_20260922_3min_official.json",
         {}, False),
    ]
    records = []
    for key, label, filename, presets, diagnosed in sources:
        path = ROOT / "frame_rate_diagnostics" / filename
        if path.exists():
            data = json.loads(path.read_text())
            records.append(dict(id=key, label=label, filename=filename, data=data,
                                presets=presets, diagnosed_zero_targets=diagnosed))
    html = (HERE / "index.html").read_text()
    for marker, source in [("SCOPE_STYLE", "style.css"), ("SCOPE_APP", "app.js"),
                           ("ECHARTS_VENDOR", "vendor/echarts.min.js"),
                           ("LUCIDE_VENDOR", "vendor/lucide.min.js")]:
        html = html.replace("/* " + marker + " */", (HERE / source).read_text())
    embedded = json.dumps({"records": records}, ensure_ascii=False, separators=(",", ":"))
    html = html.replace('{"records":[]}', embedded.replace("<", "\\u003c"))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    print(build(parser.parse_args().output))
