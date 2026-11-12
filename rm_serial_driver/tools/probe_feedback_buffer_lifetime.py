#!/usr/bin/env python3
"""Probe a supplied 38-byte firmware sender with an asynchronous CDC stub.

No serial device is opened. A sanitizer finding proves a source-level lifetime
error under deferred reads, not the cause of a particular hardware disconnect.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


STUBS = {
    "vision_link.h": """
#include <stdint.h>
#include <stddef.h>
typedef float fp32;
typedef unsigned char bool_t;
#define VISION_RX_SOF 0xA5
#define VISION_RX_DATA_LEN 8
#define VISION_RX_TAIL_1 0x5A
#define VISION_RX_TAIL_2 0x5B
#define VISION_TX_FRAME_LEN 38
#define VISION_TX_TAIL_LEN 2
#define VISION_TX_SOF 0xB5
#define VISION_TX_TAIL_1 0x5B
#define VISION_TX_TAIL_2 0x5A
uint32_t HAL_GetTick(void);
void vision_link_send_feedback(void);
""",
    "struct_typedef.h": "#include <stdint.h>\ntypedef float fp32;\ntypedef unsigned char bool_t;\n",
    "vision_aim.h": """
void vision_aim_update(float yaw, float pitch);
void vision_aim_reset_targets(void);
void vision_aim_adjust(float yaw, float pitch);
""",
    "gimbal_behaviour.h": "#include <stdint.h>\nuint8_t gimbal_get_behaviour_mode(void);\n",
    "gimbal_control.h": """
typedef struct {
    float continuous_absolute_angle;
    float absolute_angle;
    float relative_angle;
} gimbal_motor_t;
const gimbal_motor_t *get_yaw_motor_point(void);
const gimbal_motor_t *get_pitch_motor_point(void);
""",
    "INS_task.h": "#define SPI_DMA_GYRO_LENGHT 8\nconst float *get_INS_quat_point(void);\n",
    "chassis_task.h": "",
    "detect_task.h": "",
    "probe_platform.h": "#include <stdint.h>\nuint32_t HAL_GetTick(void);\n",
    "CRC8_CRC16.h": """
#include <stdint.h>
void append_CRC16_check_sum(uint8_t *data, uint32_t length);
""",
    "usbd_cdc_if.h": """
#include <stdint.h>
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length);
""",
    "probe.c": r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vision_link.h"
#include "gimbal_control.h"
static uint8_t *pending;
static uint16_t pending_length;
static gimbal_motor_t motor = {0.25f, -0.125f, -0.125f};
static unsigned updates, resets, adjustments;
bool_t vision_link_rx_byte(uint8_t byte);
static float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
const gimbal_motor_t *get_yaw_motor_point(void) { return &motor; }
const gimbal_motor_t *get_pitch_motor_point(void) { return &motor; }
const float *get_INS_quat_point(void) { return quaternion; }
uint8_t gimbal_get_behaviour_mode(void) { return 6; }
uint32_t HAL_GetTick(void) { return 5000; }
void vision_aim_update(float yaw, float pitch) { ++updates; (void)yaw; (void)pitch; }
void vision_aim_reset_targets(void) { ++resets; }
void vision_aim_adjust(float yaw, float pitch) { ++adjustments; (void)yaw; (void)pitch; }
void append_CRC16_check_sum(uint8_t *data, uint32_t length) {
    /* CRC correctness is outside this memory-lifetime probe. */
    data[length - 2] = 0; data[length - 1] = 0;
}
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length) {
    if (pending != NULL) return 1;
    pending = data; pending_length = length;
    return 0;
}
static void durable_control(void) {
    static uint8_t frame[38];
    /* Single owner: never overwrite a buffer that is still in flight. */
    if (pending != NULL) return;
    memset(frame, 0, sizeof(frame));
    frame[0] = 0xB5; frame[36] = 0x5B; frame[37] = 0x5A;
    CDC_Transmit_FS(frame, sizeof(frame));
}
int main(int argc, char **argv) {
    if (argc > 1 && (strcmp(argv[1], "intact") == 0 ||
                     strcmp(argv[1], "dropped-sof") == 0)) {
        uint8_t command[11] = {0xA5, 0, 0, 0, 0, 0, 0, 0, 0, 0x5A, 0x5B};
        unsigned start = strcmp(argv[1], "dropped-sof") == 0 ? 1 : 0;
        for (unsigned i = start; i < sizeof(command); ++i)
            vision_link_rx_byte(command[i]);
        printf("{\"updates\":%u,\"resets\":%u,\"adjustments\":%u}\n",
               updates, resets, adjustments);
        return 0;
    }
    if (argc > 1) durable_control();
    else vision_link_send_feedback();
    if (pending == NULL || pending_length != 38) return 2;
    /* Model the USB FIFO interrupt reading after the sender has returned. */
    volatile unsigned checksum = 0;
    for (unsigned i = 0; i < pending_length; ++i) checksum += pending[i];
    printf("read_bytes=%u checksum=%u\n", pending_length, checksum);
    return 0;
}
''',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True,
                        help="Supplied vision_link.c, compiled unchanged")
    parser.add_argument("--header", type=Path,
                        help="Optional actual vision_link.h instead of protocol constants stub")
    parser.add_argument("--output", type=Path,
                        help="Optional JSON report, including sanitizer details")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="rm-feedback-lifetime-") as directory:
        work = Path(directory)
        shutil.copyfile(args.source, work / "vision_link.c")
        for name, contents in STUBS.items():
            (work / name).write_text(contents)
        if args.header:
            shutil.copyfile(args.header, work / "vision_link.h")
        binary = work / "probe"
        subprocess.run([
            "gcc", "-std=c99", "-O0", "-g", "-fno-omit-frame-pointer", "-no-pie",
            "-fsanitize=address", "-include", str(work / "probe_platform.h"),
            "-I", str(work), str(work / "vision_link.c"),
            str(work / "probe.c"), "-o", str(binary),
        ], check=True, timeout=20)
        env = dict(os.environ, ASAN_OPTIONS="detect_stack_use_after_return=1:detect_leaks=0")
        original = subprocess.run([str(binary)], env=env, capture_output=True,
                                  text=True, timeout=5)
        control = subprocess.run([str(binary), "durable-control"], env=env,
                                 capture_output=True, text=True, timeout=5)
        parser_results = {}
        for mode in ("intact", "dropped-sof"):
            result = subprocess.run([str(binary), mode], env=env, capture_output=True,
                                    text=True, timeout=5, check=True)
            parser_results[mode] = json.loads(result.stdout)
        reproduced = (original.returncode != 0 and
                      "stack-use-after-return" in original.stderr)
        report = {
            "source": str(args.source.resolve()),
            "header": str(args.header.resolve()) if args.header else None,
            "scope": "Offline deferred-read stub; no USB access or flashed-firmware verification",
            "original_stack_use_after_return": reproduced,
            "durable_single_owner_control_passed": control.returncode == 0,
            "original_returncode": original.returncode,
            "sanitizer": original.stderr,
            "control_stdout": control.stdout,
            "control_stderr": control.stderr,
            "parser_results": parser_results,
        }
        if args.output:
            args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({k: v for k, v in report.items()
                          if k not in ("sanitizer", "control_stderr")}, indent=2))
        raise SystemExit(0 if reproduced and control.returncode == 0 else 1)


if __name__ == "__main__":
    main()
