from matplotlib import cm
import numpy
import sys


palette = sys.argv[1] if len(sys.argv) > 1 else "plasma"


def to_int16(c):
    r, g, b = c
    packed = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | ((b & 0b11111000) >> 3)
    return ((packed & 0xff) << 8) | (packed >> 8)

def get_palette(name):
    cmap = cm.get_cmap(name, 256)

    try:
        colors = cmap.colors
    except AttributeError:
        colors = numpy.array([cmap(i) for i in range(256)], dtype=float)

    arr = numpy.array(colors * 255).astype('uint8')
    arr = arr.reshape((16, 16, 4))
    arr = arr[:, :, 0:3]
    arr = arr.reshape((16 * 16, 3))
    print(arr)
    return [to_int16(x) for x in arr]


pal = ["0x{:04x}".format(x) for x in get_palette(palette)]

for x in range(0, 256, 16):
    pals = pal[x:x+16]
    print(", ".join(pals), end=",\n")