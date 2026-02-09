import math


def _getRainbowColor(intensity: float) -> tuple[float, float, float]:
    """
    Docstring for _getRainbowColor

    :param intensity: Description
    :type intensity: float
    :return: Description
    :rtype: tuple[float, float, float]
    """
    value = min(intensity, 1.0)
    value = max(value, 0)

    h = value * 5.0 + 1.0
    i = math.floor(h)
    f = h - i

    if (i & 1) == 0:
        f = 1 - f  # if i is even

    n = 1 - f

    r = 0
    g = 0
    b = 0

    if i <= 1:
        r = n
        g = 0
        b = 1

    elif i == 2:
        r = 0
        g = n
        b = 1

    elif i == 3:
        r = 0
        g = 1
        b = n

    elif i == 4:
        r = n
        g = 1
        b = 0

    elif i >= 5:
        r = 1
        g = n
        b = 0

    return (r, g, b)


def int2rgb(intensity: float, min: float = 0, max: float = 100) -> tuple[float, float, float]:
    """
    Converts an intensity value to an RGB color map.

    :param intensity: Intensity pixel
    :type intensity: float

    :param min: Min expected intensity pixel
    :type intensity: float

    :param max: Max expected intensity pixel
    :type intensity: float

    :return:  RGB colormap, each channel is within [1.0, 0]
    :rtype: tuple[float, float, float]
    """
    norm_i = 1.0 - (intensity - min) / (max - min)
    return _getRainbowColor(norm_i)
