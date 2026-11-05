# -*- coding: utf-8 -*-


def clamp01(value):
    if value < 0.0:
        return 0.0

    if value > 1.0:
        return 1.0

    return value


def clamp_int(value, low, high):
    if value < low:
        return low

    if value > high:
        return high

    return value


def pixel_rect_to_norm(
    x,
    y,
    width,
    height,
    image_width,
    image_height
):
    if width <= 0 or height <= 0:
        return None

    x1 = clamp01(float(x) / image_width)
    y1 = clamp01(float(y) / image_height)
    x2 = clamp01(
        float(x + width) / image_width
    )
    y2 = clamp01(
        float(y + height) / image_height
    )

    if x2 <= x1 or y2 <= y1:
        return None

    return (x1, y1, x2, y2)


def norm_rect_to_pixel(
    x1,
    y1,
    x2,
    y2,
    image_width,
    image_height
):
    x1 = clamp01(float(x1))
    y1 = clamp01(float(y1))
    x2 = clamp01(float(x2))
    y2 = clamp01(float(y2))

    if x2 <= x1 or y2 <= y1:
        return None

    px1 = clamp_int(
        int(x1 * image_width),
        0,
        image_width - 1
    )
    py1 = clamp_int(
        int(y1 * image_height),
        0,
        image_height - 1
    )
    px2 = clamp_int(
        int(x2 * image_width),
        0,
        image_width - 1
    )
    py2 = clamp_int(
        int(y2 * image_height),
        0,
        image_height - 1
    )

    if px2 <= px1 or py2 <= py1:
        return None

    return (
        px1,
        py1,
        px2 - px1,
        py2 - py1
    )


def build_target(
    x1,
    y1,
    x2,
    y2,
    label,
    score
):
    return {
        "label": int(label),
        "score": float(score),

        "x1": float(x1),
        "y1": float(y1),
        "x2": float(x2),
        "y2": float(y2),

        "cx": (
            float(x1) + float(x2)
        ) * 0.5,
        "cy": (
            float(y1) + float(y2)
        ) * 0.5,

        "w": float(x2) - float(x1),
        "h": float(y2) - float(y1),
        "bottom_y": float(y2),
    }
