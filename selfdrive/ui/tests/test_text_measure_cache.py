from collections import OrderedDict
from types import SimpleNamespace as NS

from openpilot.system.ui.lib import text_measure as tm


def setup(monkeypatch):
  calls = []
  monkeypatch.setattr(tm,'_cache',OrderedDict())
  monkeypatch.setattr(tm,'font_fallback',lambda f:f)
  monkeypatch.setattr(tm,'find_emoji',lambda text:[])
  def measure(font,text,size,spacing):
    calls.append(text)
    return NS(x=len(text)*size+spacing,y=size)
  monkeypatch.setattr(tm.rl,'measure_text_ex',measure)
  return NS(texture=NS(id=1)),calls


def test_bounded_lru_retains_recent_labels(monkeypatch):
  font,calls = setup(monkeypatch)
  monkeypatch.setattr(tm,'MAX_CACHE_ENTRIES',3)
  for text in ('a','b','c','a','d'):
    tm.measure_text_cached(font,text,20)
  assert len(tm._cache) == 3
  assert calls == ['a','b','c','d']
  tm.measure_text_cached(font,'b',20)
  assert calls[-1] == 'b'


def test_hash_collision_cannot_reuse_wrong_width(monkeypatch):
  class Colliding(str):
    def __hash__(self):
      return 42
  font,_ = setup(monkeypatch)
  a = tm.measure_text_cached(font,Colliding('a'),20)
  b = tm.measure_text_cached(font,Colliding('longer'),20)
  assert b.x == a.x*6


def test_long_text_not_retained_and_spacing_part_of_key(monkeypatch):
  font,calls = setup(monkeypatch)
  tm.measure_text_cached(font,'x'*(tm.MAX_CACHE_TEXT_LENGTH+1),20)
  assert not tm._cache
  a = tm.measure_text_cached(font,'abc',20,0)
  b = tm.measure_text_cached(font,'abc',20,2)
  assert b.x == a.x+2 and len(calls) == 3


def test_emoji_measurement_remains_identical(monkeypatch):
  font,_ = setup(monkeypatch)
  monkeypatch.setattr(tm,'find_emoji',lambda text:[(0,1,None)])
  result = tm.measure_text_cached(font,'X',20)
  assert result.x == 20*tm.FONT_SCALE and result.y == 20*tm.FONT_SCALE
