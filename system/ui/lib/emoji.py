import io
import re

from PIL import Image, ImageDraw, ImageFont
import pyray as rl

from openpilot.system.ui.lib.application import FONT_DIR

_emoji_font: ImageFont.FreeTypeFont | None = None
_cache: dict[str, rl.Texture] = {}

EMOJI_REGEX = re.compile(
  """[\U0001f600-\U0001f64f
\U0001f300-\U0001f5ff
\U0001f680-\U0001f6ff
\U0001f1e0-\U0001f1ff
\U00002700-\U000027bf
\U0001f900-\U0001f9ff
\U00002600-\U000026ff
\U00002300-\U000023ff
\U00002b00-\U00002bff
\U0001fa70-\U0001faff
\U0001f700-\U0001f77f
\u2640-\u2642
\u2600-\u2b55
\u200d
\u23cf
\u23e9
\u231a
\ufe0f
\u3030
]+""".replace("\n", ""),
  flags=re.UNICODE,
)


def _load_emoji_font() -> ImageFont.FreeTypeFont | None:
  global _emoji_font
  if _emoji_font is None:
    _emoji_font = ImageFont.truetype(str(FONT_DIR.joinpath("NotoColorEmoji.ttf")), 109)
  return _emoji_font


def find_emoji(text):
  return [(m.start(), m.end(), m.group()) for m in EMOJI_REGEX.finditer(text)]


def emoji_tex(emoji):
  if emoji not in _cache:
    img = Image.new("RGBA", (128, 128), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), emoji, font=_load_emoji_font(), embedded_color=True)
    with io.BytesIO() as buffer:
      img.save(buffer, format="PNG")
      l = buffer.tell()
      buffer.seek(0)
      _cache[emoji] = rl.load_texture_from_image(rl.load_image_from_memory(".png", buffer.getvalue(), l))
  return _cache[emoji]
