import math
import time
from dataclasses import dataclass, replace

from opendbc.can import CANParser
from opendbc.can.dbc import DBC as DBCReader
from opendbc.can.parser import get_raw_value
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags, HYUNDAI_MANDO_FRONT_RADAR_DBC, HYUNDAI_MRREVO14F_RADAR_DBC, \
                                       HYUNDAI_MRR30_RADAR_DBC, HYUNDAI_MRR35_RADAR_DBC
from openpilot.common.swaglog import cloudlog

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
G90_RADAR_MSG_COUNT = 64
MRREVO14F_RADAR_START_ADDR = 0x602
MRREVO14F_RADAR_MSG_COUNT = 16
MRR30_RADAR_START_ADDR = 0x210
MRR30_RADAR_MSG_COUNT = 16
MRR35_RADAR_START_ADDR = 0x3A5
MRR35_RADAR_MSG_COUNT = 32
CARNIVAL_4TH_GEN_OBJECT_START_ADDR = 0x180
CARNIVAL_4TH_GEN_OBJECT_END_ADDR = 0x184
CARNIVAL_4TH_GEN_PRIMARY_OBJECT_ADDR = 0x180
CARNIVAL_4TH_GEN_PRIMARY_OBJECT_SLOT_OFFSET = 0
CARNIVAL_4TH_GEN_OBJECT_BUS = 1
CARNIVAL_4TH_GEN_OBJECT_LEN = 32
CARNIVAL_4TH_GEN_OBJECT_LOG_INTERVAL = 1.0
CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_BASE = 0xC4100
CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE = 0.25
CARNIVAL_4TH_GEN_CONFIRMATION_MIN_PERSIST = 3
CARNIVAL_4TH_GEN_CONFIRMATION_MAX_GAP = 0.15
CARNIVAL_4TH_GEN_CONFIRMATION_VELOCITY_SPEC = (91, 11, 0.05, 2.4)
CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_Y = 50.0
CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_V = 60.0


def get_little_unsigned(dat: bytes, start: int, size: int) -> int:
  return (int.from_bytes(dat, "little", signed=False) >> start) & ((1 << size) - 1)


def get_little_signed(dat: bytes, start: int, size: int) -> int:
  val = get_little_unsigned(dat, start, size)
  return val - (1 << size) if val & (1 << (size - 1)) else val


def decode_carnival_confirmation_velocity(dat: bytes, bit_offset: int, _state: int, _state_alt: int) -> float:
  # Verified against 15,382 stock-SCC samples from the Carnival's 99110-R0100
  # radar: median 0.15 m/s and p95 0.55 m/s versus ACC_ObjRelSpd.
  start, size, scale, offset = CARNIVAL_4TH_GEN_CONFIRMATION_VELOCITY_SPEC
  return get_little_signed(dat, bit_offset + start, size) * scale + offset


@dataclass(frozen=True)
class CarnivalRadarObject:
  raw_track_id: int
  heartbeat: int
  d_rel: float
  y_rel: float
  v_rel: float


def decode_carnival_radar_object(dat: bytes, bit_offset: int) -> CarnivalRadarObject:
  return CarnivalRadarObject(
    raw_track_id=get_little_unsigned(dat, bit_offset + 42, 8),
    heartbeat=get_little_unsigned(dat, bit_offset + 124, 4),
    d_rel=get_little_unsigned(dat, bit_offset + 64, 13) * 0.05,
    y_rel=get_little_signed(dat, bit_offset + 78, 11) * 0.05,
    v_rel=decode_carnival_confirmation_velocity(dat, bit_offset, 0, 0),
  )


def carnival_radar_object_valid(obj: CarnivalRadarObject) -> bool:
  return (obj.raw_track_id != 0 and obj.heartbeat != 0 and
          0.5 <= obj.d_rel <= 220.0 and
          abs(obj.y_rel) <= CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_Y and
          abs(obj.v_rel) <= CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_V)


def carnival_confirmation_continuous(prev: tuple[float, float, float, float] | None, now: float,
                                     obj: CarnivalRadarObject) -> bool:
  if prev is None:
    return False
  prev_t, prev_d, prev_y, prev_v = prev
  dt = now - prev_t
  return (0.0 <= dt <= CARNIVAL_4TH_GEN_CONFIRMATION_MAX_GAP and
          abs(obj.d_rel - prev_d) <= max(1.5, 60.0 * max(dt, 0.0)) and
          abs(obj.y_rel - prev_y) <= max(1.0, 20.0 * max(dt, 0.0)) and
          abs(obj.v_rel - prev_v) <= 8.0)


@dataclass(frozen=True)
class RadarTrackConfig:
  start_addr: int
  msg_count: int
  radar_type: str
  bus: int = 1
  frequency: int = 50
  parser_msg_count: int | None = None
  expected_length: int | None = None

  @property
  def can_parser_msg_count(self) -> int:
    return self.parser_msg_count if self.parser_msg_count is not None else self.msg_count


RADAR_TRACK_CONFIGS = {
  HYUNDAI_MANDO_FRONT_RADAR_DBC: RadarTrackConfig(RADAR_START_ADDR, RADAR_MSG_COUNT, "mando"),
  HYUNDAI_MRREVO14F_RADAR_DBC: RadarTrackConfig(MRREVO14F_RADAR_START_ADDR, MRREVO14F_RADAR_MSG_COUNT, "mrrevo14f"),
  HYUNDAI_MRR30_RADAR_DBC: RadarTrackConfig(MRR30_RADAR_START_ADDR, MRR30_RADAR_MSG_COUNT, "mrr30", bus=0, expected_length=32),
  HYUNDAI_MRR35_RADAR_DBC: RadarTrackConfig(MRR35_RADAR_START_ADDR, MRR35_RADAR_MSG_COUNT, "mrr35", bus=0, frequency=20, expected_length=24),
}

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_track_config(car_fingerprint, flags: int = 0) -> RadarTrackConfig | None:
  radar_dbc = DBC[car_fingerprint].get(Bus.radar)
  if car_fingerprint == CAR.GENESIS_G90 and radar_dbc == HYUNDAI_MANDO_FRONT_RADAR_DBC:
    return RadarTrackConfig(RADAR_START_ADDR, G90_RADAR_MSG_COUNT, "mando", parser_msg_count=RADAR_MSG_COUNT)

  radar_config = RADAR_TRACK_CONFIGS.get(radar_dbc)
  if radar_config is None:
    return None

  if car_fingerprint == CAR.HYUNDAI_IONIQ_6 and flags & HyundaiFlags.CANFD_CAMERA_SCC:
    return replace(radar_config, bus=1)

  return radar_config


def radar_tracks_available(radar_config: RadarTrackConfig | None, fingerprint) -> bool:
  if radar_config is None:
    return False

  msg_len = fingerprint[radar_config.bus].get(radar_config.start_addr)
  if msg_len is None:
    return False

  return radar_config.expected_length is None or msg_len == radar_config.expected_length


def get_radar_can_parser(CP, radar_config):
  if radar_config is None:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", radar_config.frequency)
              for addr in range(radar_config.start_addr, radar_config.start_addr + radar_config.can_parser_msg_count)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, radar_config.bus)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.radar_config = get_radar_track_config(CP.carFingerprint, CP.flags)
    self.updated_messages = set()
    self.trigger_msg = (self.radar_config.start_addr + self.radar_config.can_parser_msg_count - 1
                        if self.radar_config is not None else RADAR_START_ADDR)
    self.track_id = 0
    self.g90_extended_mando = (CP.carFingerprint == CAR.GENESIS_G90 and self.radar_config is not None and
                               self.radar_config.msg_count > self.radar_config.can_parser_msg_count)
    self.g90_mando_signals = []
    if self.g90_extended_mando:
      radar_dbc = DBCReader(DBC[CP.carFingerprint][Bus.radar])
      self.g90_mando_signals = list(radar_dbc.addr_to_msg[RADAR_START_ADDR].sigs.values())

    self.radar_off_can = CP.radarUnavailable
    # Probe whether radar tracks still exist on the Ioniq 6 while OP long is active,
    # without changing planner behavior yet.
    self.ioniq_6_radar_probe = CP.carFingerprint == CAR.HYUNDAI_IONIQ_6 and CP.openpilotLongitudinalControl and self.radar_off_can
    self.ioniq_6_radar_probe_logged = False
    self.ioniq_6_radar_probe_updates = 0
    # The 2024 Carnival exposes an MRR30-like object bank at 0x180-0x184 on bus 1.
    # Keep this as a shadow probe until lateral/velocity fields are fully decoded;
    # publishing it as liveTracks would immediately affect longitudinal planning.
    self.carnival_object_probe = CP.carFingerprint == CAR.KIA_CARNIVAL_4TH_GEN
    self.carnival_object_probe_prev: dict[tuple[int, int], tuple[float, float]] = {}
    self.carnival_object_probe_last_log = 0.0
    self.carnival_object_probe_seen = 0
    self.carnival_object_probe_valid = 0
    self.carnival_confirmation_tracks: dict[int, tuple[float, float, float, float]] = {}
    self.carnival_confirmation_prev: dict[int, tuple[float, float, float, float]] = {}
    self.carnival_confirmation_persist: dict[int, int] = {}
    self.rcp = get_radar_can_parser(CP, self.radar_config)

    # Precompute (addr, "RADAR_TRACK_xxx") pairs once. _update runs on the
    # CAN-driven card loop (core 4, shared with controlsd/selfdrived), so avoid
    # rebuilding 32 f-strings per radar frame.
    self.track_addrs: list[tuple[int, str]] = []
    if self.radar_config is not None:
      self.track_addrs = [(addr, f"RADAR_TRACK_{addr:x}")
                          for addr in range(self.radar_config.start_addr,
                                            self.radar_config.start_addr + self.radar_config.can_parser_msg_count)]

  def update(self, can_strings):
    if self.carnival_object_probe:
      self._update_carnival_object_probe(can_strings)

    if self.ioniq_6_radar_probe and self.rcp is not None and not self.ioniq_6_radar_probe_logged:
      vls = self.rcp.update(can_strings)
      self.updated_messages.update(vls)
      self.ioniq_6_radar_probe_updates += 1

      if self.trigger_msg in self.updated_messages:
        rr = self._update(self.updated_messages)
        cloudlog.warning(f"Ioniq 6 radar probe: saw {len(rr.points)} radar tracks with radarUnavailable forced on")
        self.updated_messages.clear()
        self.ioniq_6_radar_probe_logged = True
      elif self.ioniq_6_radar_probe_updates >= 500:
        cloudlog.warning("Ioniq 6 radar probe: no radar track frames observed after startup")
        self.ioniq_6_radar_probe_logged = True
        self.updated_messages.clear()

    if self.radar_off_can or (self.rcp is None):
      rr = super().update(None)
      if rr is not None and self.carnival_object_probe:
        self._add_carnival_confirmation_tracks(rr)
      return rr

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)
    if self.g90_extended_mando:
      self._update_g90_extended_mando_tracks(can_strings)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    if self.carnival_object_probe:
      self._add_carnival_confirmation_tracks(rr)

    return rr

  def _update_carnival_object_probe(self, can_strings):
    now = time.monotonic()
    primary: CarnivalRadarObject | None = None
    batch_objects: dict[int, CarnivalRadarObject] = {}
    conflicting_ids: set[int] = set()

    for _, frames in can_strings:
      for address, dat, src in frames:
        if src != CARNIVAL_4TH_GEN_OBJECT_BUS:
          continue
        if not (CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR):
          continue
        if len(dat) != CARNIVAL_4TH_GEN_OBJECT_LEN:
          continue

        self.carnival_object_probe_seen += 1
        for bit_offset in (0, 128):
          obj = decode_carnival_radar_object(dat, bit_offset)
          if not carnival_radar_object_valid(obj):
            continue
          self.carnival_object_probe_valid += 1
          if address == CARNIVAL_4TH_GEN_PRIMARY_OBJECT_ADDR and bit_offset == CARNIVAL_4TH_GEN_PRIMARY_OBJECT_SLOT_OFFSET:
            primary = obj
          previous_in_batch = batch_objects.get(obj.raw_track_id)
          if previous_in_batch is not None and previous_in_batch != obj:
            conflicting_ids.add(obj.raw_track_id)
          else:
            batch_objects[obj.raw_track_id] = obj

    for raw_track_id in conflicting_ids:
      batch_objects.pop(raw_track_id, None)
      self.carnival_confirmation_tracks.pop(raw_track_id, None)
      self.carnival_confirmation_prev.pop(raw_track_id, None)
      self.carnival_confirmation_persist.pop(raw_track_id, None)

    for raw_track_id, obj in batch_objects.items():
      previous = self.carnival_confirmation_prev.get(raw_track_id)
      continuous = carnival_confirmation_continuous(previous, now, obj)
      persist = self.carnival_confirmation_persist.get(raw_track_id, 0) + 1 if continuous else 1
      if not continuous:
        # A raw ID can be recycled for a different physical object. Withdraw the
        # old point immediately so radard creates a fresh Track after persistence
        # is re-established instead of inheriting its maturity and path history.
        self.carnival_confirmation_tracks.pop(raw_track_id, None)
      self.carnival_confirmation_prev[raw_track_id] = (now, obj.d_rel, obj.y_rel, obj.v_rel)
      self.carnival_confirmation_persist[raw_track_id] = persist
      if persist >= CARNIVAL_4TH_GEN_CONFIRMATION_MIN_PERSIST:
        self.carnival_confirmation_tracks[raw_track_id] = (now, obj.d_rel, obj.y_rel, obj.v_rel)

    self._expire_carnival_confirmation_tracks(now)
    if primary is None or now - self.carnival_object_probe_last_log < CARNIVAL_4TH_GEN_OBJECT_LOG_INTERVAL:
      return

    primary_prev = self.carnival_object_probe_prev.get((CARNIVAL_4TH_GEN_PRIMARY_OBJECT_ADDR, 1))
    d_dot = float("nan")
    if primary_prev is not None:
      prev_t, prev_d = primary_prev
      dt = now - prev_t
      if 0.01 <= dt <= 1.0:
        d_dot = (primary.d_rel - prev_d) / dt
    self.carnival_object_probe_prev[(CARNIVAL_4TH_GEN_PRIMARY_OBJECT_ADDR, 1)] = (now, primary.d_rel)
    d_dot_str = "nan" if not math.isfinite(d_dot) else f"{d_dot:.2f}"
    cloudlog.warning("".join((
      "Carnival 4th gen radar probe: ",
      f"addr=0x{CARNIVAL_4TH_GEN_PRIMARY_OBJECT_ADDR:x} slot=1 rawTrackId={primary.raw_track_id} ",
      f"heartbeat={primary.heartbeat} dRel={primary.d_rel:.2f} yRel={primary.y_rel:.2f} ",
      f"vRel={primary.v_rel:.2f} dDot={d_dot_str} shadowDistance=YES ",
      f"confirmationTrack={primary.raw_track_id in self.carnival_confirmation_tracks} ",
      f"publishedTracks={len(self.carnival_confirmation_tracks)} publishReady={bool(self.carnival_confirmation_tracks)} ",
      f"velocityDecoded=YES controlReady=BOUNDED_CONFIRMATION_BLEND conflicts={len(conflicting_ids)} ",
      f"seen={self.carnival_object_probe_seen} ",
      f"valid={self.carnival_object_probe_valid}",
    )))
    self.carnival_object_probe_last_log = now

  def _expire_carnival_confirmation_tracks(self, now):
    stale = [raw_track_id for raw_track_id, (track_time, *_) in self.carnival_confirmation_tracks.items()
             if now - track_time > CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE]
    for raw_track_id in stale:
      self.carnival_confirmation_tracks.pop(raw_track_id, None)
      self.carnival_confirmation_prev.pop(raw_track_id, None)
      self.carnival_confirmation_persist.pop(raw_track_id, None)

  def _add_carnival_confirmation_tracks(self, rr):
    self._expire_carnival_confirmation_tracks(time.monotonic())
    if not self.carnival_confirmation_tracks:
      return

    points = list(rr.points)
    for raw_track_id, (_, d_rel, y_rel, v_rel) in sorted(self.carnival_confirmation_tracks.items()):
      pt = structs.RadarData.RadarPoint()
      pt.trackId = CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_BASE + raw_track_id
      pt.measured = True
      pt.dRel = float(d_rel)
      pt.yRel = float(y_rel)
      pt.vRel = float(v_rel)
      pt.aRel = float("nan")
      pt.yvRel = float("nan")
      points.append(pt)
    rr.points = points

  def _decode_g90_mando_values(self, dat: bytes):
    vals = {}
    for sig in self.g90_mando_signals:
      raw = get_raw_value(dat, sig)
      if sig.is_signed:
        raw -= ((raw >> (sig.size - 1)) & 1) * (1 << sig.size)
      vals[sig.name] = raw * sig.factor + sig.offset
    return vals

  def _update_g90_extended_mando_tracks(self, can_strings):
    if self.radar_config is None:
      return

    start_addr = self.radar_config.start_addr + self.radar_config.can_parser_msg_count
    end_addr = self.radar_config.start_addr + self.radar_config.msg_count

    for _, frames in can_strings:
      for address, dat, src in frames:
        if src != self.radar_config.bus or not (start_addr <= address < end_addr) or len(dat) < 8:
          continue

        self.updated_messages.add(address)
        msg = self._decode_g90_mando_values(dat)
        valid = msg["STATE"] in (3, 4)
        if valid:
          if address not in self.pts:
            self.pts[address] = structs.RadarData.RadarPoint()
            self.pts[address].trackId = self.track_id
            self.track_id += 1

          azimuth = math.radians(msg["AZIMUTH"])
          self.pts[address].measured = True
          self.pts[address].dRel = math.cos(azimuth) * msg["LONG_DIST"]
          self.pts[address].yRel = 0.5 * -math.sin(azimuth) * msg["LONG_DIST"]
          self.pts[address].vRel = msg["REL_SPEED"]
          self.pts[address].aRel = msg["REL_ACCEL"]
          self.pts[address].yvRel = float("nan")
        elif address in self.pts:
          del self.pts[address]

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.radar_config is None:
      return ret

    radar_type = self.radar_config.radar_type
    vl = self.rcp.vl

    for addr, track_name in self.track_addrs:
      msg = vl[track_name]

      if radar_type == "mrr30":
        for i in ("1", "2"):
          track_key = addr * 2 + int(i) - 1
          if track_key not in self.pts:
            self.pts[track_key] = structs.RadarData.RadarPoint()
            self.pts[track_key].trackId = self.track_id
            self.track_id += 1

          valid = msg[f"{i}_STATE"] in (3, 4)
          if valid:
            pt = self.pts[track_key]
            pt.measured = True
            pt.dRel = msg[f"{i}_LONG_DIST"]
            pt.yRel = msg[f"{i}_LAT_DIST"]
            pt.vRel = msg[f"{i}_REL_SPEED"]
            pt.aRel = float("nan")
            pt.yvRel = float("nan")
          else:
            del self.pts[track_key]
        continue

      if radar_type == "mrrevo14f":
        for i in ("1", "2"):
          track_key = addr * 2 + int(i) - 1
          valid = msg[f"{i}_DISTANCE"] != 255.75
          if valid:
            pt = self.pts.get(track_key)
            if pt is None:
              pt = structs.RadarData.RadarPoint()
              pt.trackId = self.track_id
              self.track_id += 1
              self.pts[track_key] = pt
            pt.measured = True
            pt.dRel = msg[f"{i}_DISTANCE"]
            pt.yRel = msg[f"{i}_LATERAL"]
            pt.vRel = msg[f"{i}_SPEED"]
            pt.aRel = float("nan")
            pt.yvRel = float("nan")
          elif track_key in self.pts:
            del self.pts[track_key]
        continue

      if radar_type == "mrr35":
        # Most of the 32 channels are empty each frame. Only allocate a point
        # when the channel is valid; drop it otherwise. Avoids the per-frame
        # alloc-then-delete churn on the ~27 idle channels.
        if msg["STATE"] in (3, 4):
          pt = self.pts.get(addr)
          if pt is None:
            pt = structs.RadarData.RadarPoint()
            pt.trackId = self.track_id
            self.track_id += 1
            self.pts[addr] = pt
          pt.measured = True
          pt.dRel = msg["LONG_DIST"]
          pt.yRel = msg["LAT_DIST"]
          pt.vRel = msg["REL_SPEED"]
          pt.aRel = msg["REL_ACCEL"]
          pt.yvRel = float("nan")
        elif addr in self.pts:
          del self.pts[addr]
        continue

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].measured = True
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']
        self.pts[addr].aRel = msg['REL_ACCEL']
        self.pts[addr].yvRel = float('nan')

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
