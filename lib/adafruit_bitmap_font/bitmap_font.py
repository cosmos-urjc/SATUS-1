# SPDX-FileCopyrightText: 2019 Scott Shawcroft for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_bitmap_font.bitmap_font`
====================================================

Loads bitmap glyphs from a variety of font.

* Author(s): Scott Shawcroft

Implementation Notes
--------------------

**Hardware:**

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

"""

try:
    from typing import Optional, Union
    from displayio import Bitmap
    from . import bdf
    from . import pcf
    from . import ttf
except ImportError:
    pass

__version__ = "2.1.4"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_Bitmap_Font.git"


def load_font(
    filename: str, bitmap: Optional[Bitmap] = None
) -> Union[bdf.BDF, pcf.PCF, ttf.TTF]:
    """Loads a font file. Returns None if unsupported."""
    # pylint: disable=import-outside-toplevel, redefined-outer-name, consider-using-with
    if not bitmap:
        import displayio

        bitmap = displayio.Bitmap
    font_file = open(filename, "rb")
    first_four = font_file.read(4)
    if filename.endswith("bdf") and first_four == b"STAR":
        from . import bdf

        return bdf.BDF(font_file, bitmap)
    if filename.endswith("pcf") and first_four == b"\x01fcp":
        from . import pcf

        return pcf.PCF(font_file, bitmap)
    if filename.endswith("ttf") and first_four == b"\x00\x01\x00\x00":
        from . import ttf

        return ttf.TTF(font_file, bitmap)

    raise ValueError("Unknown magic number %r" % first_four)
