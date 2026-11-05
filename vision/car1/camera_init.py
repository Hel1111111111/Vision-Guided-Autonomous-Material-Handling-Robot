# -*- coding: utf-8 -*-
import sensor
import time

import camera_profile


def _set_exposure(exposure_us):
    try:
        sensor.set_auto_exposure(
            False,
            exposure_us=int(exposure_us)
        )
    except TypeError:
        sensor.set_auto_exposure(
            False,
            int(exposure_us)
        )


def _set_gain(gain_db):
    try:
        sensor.set_auto_gain(
            False,
            gain_db=float(gain_db)
        )
    except TypeError:
        sensor.set_auto_gain(
            False,
            float(gain_db)
        )


def _set_white_balance(rgb_gain_db):
    try:
        sensor.set_auto_whitebal(
            False,
            rgb_gain_db=rgb_gain_db
        )
    except TypeError:
        sensor.set_auto_whitebal(
            False,
            rgb_gain_db
        )


def init_camera(profile=None):
    if profile is None:
        profile = camera_profile.FIXED_STADIUM_PROFILE

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)

    try:
        sensor.set_brightness(0)
        sensor.set_contrast(0)
        sensor.set_saturation(0)
    except Exception as error:
        print("IMAGE PARAM WARN:", error)

    _set_exposure(profile["exposure_us"])
    _set_gain(profile["gain_db"])
    _set_white_balance(profile["rgb_gain_db"])

    sensor.skip_frames(
        time=camera_profile.CAMERA_SETTLE_MS
    )

    print("================================")
    print("CAMERA PROFILE FIXED")
    print("EXPOSURE_US:", profile["exposure_us"])
    print("GAIN_DB:", profile["gain_db"])
    print("RGB_GAIN_DB:", profile["rgb_gain_db"])
    print("================================")

    return time.clock()
