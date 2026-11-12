#!/usr/bin/env python3
"""Read and validate the MVS frame-rate range without changing camera state."""

import argparse
import json
import math
import sys
from ctypes import POINTER, c_bool, cast


MVS_IMPORT = "/opt/MVS/Samples/64/Python/MvImport"
if MVS_IMPORT not in sys.path:
    sys.path.insert(0, MVS_IMPORT)

from MvCameraControl_class import (  # noqa: E402,F403
    MV_ACCESS_Monitor,
    MV_CC_DEVICE_INFO,
    MV_CC_DEVICE_INFO_LIST,
    MV_USB_DEVICE,
    MVCC_ENUMVALUE,
    MVCC_FLOATVALUE,
    MVCC_INTVALUE_EX,
    MvCamera,
    PixelType_Gvsp_BayerRG8,
    PixelType_Gvsp_BGR8_Packed,
)


def decode(value):
    return bytes(value).split(b"\0", 1)[0].decode("ascii", errors="replace")


def require(result, operation):
    if result != 0:
        raise RuntimeError(f"{operation} failed: 0x{result:08x}")


def float_value(camera, name):
    value = MVCC_FLOATVALUE()
    require(camera.MV_CC_GetFloatValue(name, value), f"get {name}")
    return {
        "current": float(value.fCurValue),
        "minimum": float(value.fMin),
        "maximum": float(value.fMax),
    }


def int_value(camera, name):
    value = MVCC_INTVALUE_EX()
    require(camera.MV_CC_GetIntValueEx(name, value), f"get {name}")
    return int(value.nCurValue)


def enum_value(camera, name):
    value = MVCC_ENUMVALUE()
    require(camera.MV_CC_GetEnumValue(name, value), f"get {name}")
    return int(value.nCurValue)


def bool_value(camera, name):
    value = c_bool(False)
    require(camera.MV_CC_GetBoolValue(name, value), f"get {name}")
    return bool(value.value)


PIXEL_FORMATS = {
    "BayerRG8": PixelType_Gvsp_BayerRG8,
    "BGR8": PixelType_Gvsp_BGR8_Packed,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="DA8541046")
    parser.add_argument("--width", type=int, default=1440)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--exposure-us", type=float, default=3000.0)
    parser.add_argument("--gain-db", type=float, default=12.0)
    parser.add_argument(
        "--pixel-format", choices=sorted(PIXEL_FORMATS), default="BayerRG8"
    )
    args = parser.parse_args()

    camera = None
    opened = False
    require(MvCamera.MV_CC_Initialize(), "initialize SDK")
    try:
        devices = MV_CC_DEVICE_INFO_LIST()
        require(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, devices), "enumerate USB cameras")
        selected = None
        model = ""
        serial = ""
        for index in range(devices.nDeviceNum):
            device = cast(devices.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)).contents
            candidate_serial = decode(device.SpecialInfo.stUsb3VInfo.chSerialNumber)
            if candidate_serial == args.serial:
                selected = device
                serial = candidate_serial
                model = decode(device.SpecialInfo.stUsb3VInfo.chModelName)
                break
        if selected is None:
            raise RuntimeError(
                f"camera {args.serial} not found; enumerated {devices.nDeviceNum} USB camera(s)"
            )

        camera = MvCamera()
        require(camera.MV_CC_CreateHandle(selected), "create handle")
        require(camera.MV_CC_OpenDevice(MV_ACCESS_Monitor, 0), "open device read-only")
        opened = True

        width = int_value(camera, "Width")
        height = int_value(camera, "Height")
        pixel_format = enum_value(camera, "PixelFormat")
        exposure = float_value(camera, "ExposureTime")
        gain = float_value(camera, "Gain")
        frame_rate_enabled = bool_value(camera, "AcquisitionFrameRateEnable")
        expected_pixel_format = PIXEL_FORMATS[args.pixel_format]
        checks = {
            "width": width == args.width,
            "height": height == args.height,
            "pixel_format": pixel_format == expected_pixel_format,
            "exposure_time_us": math.isclose(
                exposure["current"], args.exposure_us, abs_tol=1e-3
            ),
            "gain_db": math.isclose(gain["current"], args.gain_db, abs_tol=1e-3),
            "acquisition_frame_rate_enabled": frame_rate_enabled,
        }

        output = {
            "read_only": True,
            "identity": {"model": model, "serial": serial},
            "expected_configuration": {
                "width": args.width,
                "height": args.height,
                "pixel_format": {
                    "name": args.pixel_format,
                    "value": expected_pixel_format,
                },
                "exposure_time_us": args.exposure_us,
                "gain_db": args.gain_db,
                "acquisition_frame_rate_enabled": True,
            },
            "configuration": {
                "width": width,
                "height": height,
                "pixel_format": pixel_format,
                "exposure_time_us": exposure,
                "gain_db": gain,
                "acquisition_frame_rate_enabled": frame_rate_enabled,
            },
            "acquisition_frame_rate_hz": float_value(camera, "AcquisitionFrameRate"),
            "resulting_frame_rate_hz": float_value(camera, "ResultingFrameRate"),
            "checks": checks,
            "passed": all(checks.values()),
        }
        print(json.dumps(output, indent=2, sort_keys=True))
        if not output["passed"]:
            raise SystemExit(2)
    finally:
        if camera is not None:
            if opened:
                camera.MV_CC_CloseDevice()
            camera.MV_CC_DestroyHandle()
        MvCamera.MV_CC_Finalize()


if __name__ == "__main__":
    main()
