from matplotlib import cm
import numpy

def to_int16(c):
    r, g, b = c
    return ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | ((b & 0b1111100) >> 3)


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
    return [to_int16(x) for x in arr]


pal = ["0x{:04x}".format(x) for x in get_palette("plasma")]

for x in range(0, 256, 16):
    pals = pal[x:x+16]
    print(", ".join(pals), end=",\n")