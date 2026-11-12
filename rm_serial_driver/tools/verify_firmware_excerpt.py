#!/usr/bin/env python3
"""Compile a supplied vision_link excerpt against memory-only host stubs."""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


STUBS = {
    "struct_typedef.h": "typedef float fp32; typedef unsigned char bool_t;\n",
    "vision_aim.h": "void vision_aim_update(float yaw, float pitch);\n",
    "gimbal_behaviour.h": """
#include <stdint.h>
typedef struct { float continuous_absolute_angle; } gimbal_motor_t;
const gimbal_motor_t *get_yaw_motor_point(void);
uint8_t gimbal_get_behaviour_mode(void);
""",
    "usbd_cdc_if.h": """
#include <stdint.h>
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t size);
""",
    "probe.c": r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vision_link.h"
#include "gimbal_behaviour.h"
static unsigned updates, attempts, size_out;
static float received_yaw, received_pitch;
static uint8_t output[128];
static gimbal_motor_t motor = {0.5f};
const gimbal_motor_t *get_yaw_motor_point(void) { return &motor; }
uint8_t gimbal_get_behaviour_mode(void) { return 3; }
void vision_aim_update(float yaw, float pitch) {
    ++updates; received_yaw = yaw; received_pitch = pitch;
}
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t size) {
    ++attempts;
    if (size > sizeof(output)) return 1;
    memcpy(output, data, size); size_out = size;
    return 0;
}
int main(void) {
    uint8_t wire[11] = {0xA5};
    float yaw = 0.25f, pitch = -0.125f;
    memcpy(wire + 1, &yaw, 4); memcpy(wire + 5, &pitch, 4);
    wire[9] = 0x5A; wire[10] = 0x5B;
    for (unsigned i = 0; i < sizeof(wire); ++i) vision_link_rx_byte(wire[i]);
    printf("{\"mode\":3,\"input_bytes\":11,\"parsed_commands\":%u,"
           "\"send_attempts\":%u,\"response_bytes\":%u,"
           "\"yaw\":%.6f,\"pitch\":%.6f}\n",
           updates, attempts, size_out, received_yaw, received_pitch);
    return !(updates == 1 && attempts == 1 && size_out == 8 &&
             received_yaw == yaw && received_pitch == pitch &&
             output[0] == 0xB5 && output[5] == 3 &&
             output[6] == 0x5B && output[7] == 0x5A);
}
''',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="rm-firmware-excerpt-") as directory:
        work = Path(directory)
        for extension in ("c", "h"):
            source = args.source_dir / f"vision_link(1).{extension}"
            if not source.is_file():
                source = args.source_dir / f"vision_link.{extension}"
            shutil.copyfile(source, work / f"vision_link.{extension}")
        for name, content in STUBS.items():
            (work / name).write_text(content)
        binary = work / "probe"
        subprocess.run(["cc", "-std=c99", "-include", "stdint.h", "-I", str(work),
                        str(work / "vision_link.c"), str(work / "probe.c"),
                        "-o", str(binary)], check=True, timeout=20)
        result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=5)
        report = json.loads(result.stdout)
        report["scope"] = "supplied source excerpt only; no USB device or flashed firmware tested"
        report["passed"] = result.returncode == 0
        print(json.dumps(report, indent=2))
        raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
