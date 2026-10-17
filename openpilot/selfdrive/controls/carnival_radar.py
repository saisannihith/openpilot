"""Model-first lead qualification for the 2022-24 Carnival R0100 radar bank."""

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


RADAR_TO_CAMERA = 1.52
TRACK_ID_MIN = 0xC4100
TRACK_ID_MAX = 0xC41FF


@dataclass
class Evidence:
  count: int = 0
  path_frames: int = 0
  distances: deque[tuple[float, float]] = field(default_factory=lambda: deque(maxlen=32))


class CarnivalRadarPolicy:
  def __init__(self):
    self.evidence: dict[int, Evidence] = {}
    self.selected = [-1, -1]
    self.selected_frames = [0, 0]

  def update(self, tracks, *, fresh: bool, sample_time: float) -> None:
    for identifier in tuple(self.evidence):
      if identifier not in tracks:
        del self.evidence[identifier]
    if not fresh:
      return
    for identifier, track in tracks.items():
      if not TRACK_ID_MIN <= identifier <= TRACK_ID_MAX:
        continue
      evidence = self.evidence.setdefault(identifier, Evidence())
      if evidence.distances and (sample_time <= evidence.distances[-1][0] or
                                 sample_time - evidence.distances[-1][0] > 0.15):
        evidence.count = 0
        evidence.path_frames = 0
        evidence.distances.clear()
      evidence.count += 1
      evidence.distances.append((sample_time, track.dRel))

  @staticmethod
  def _score(track, lead, model_v_ego: float) -> float:
    try:
      residual_d = float(track.dRel) - (float(lead.x[0]) - RADAR_TO_CAMERA)
      residual_y = float(track.yRel) + float(lead.y[0])
      moving = float(track.vRel) + model_v_ego > 3.0
      residual_v = 0.0 if moving else float(track.vRel) - (float(lead.v[0]) - model_v_ego)
      if (abs(residual_d) >= max(abs(float(lead.x[0]) - RADAR_TO_CAMERA) * 0.22, 4.0) or
          abs(residual_y) >= max(1.2, 1.5 * max(float(lead.yStd[0]), 0.2)) or
          (not moving and abs(residual_v) >= max(4.0, 3.0 * max(float(lead.vStd[0]), 0.5))) or
          float(track.vRel) + model_v_ego <= -2.0):
        return math.inf
      sd = float(np.clip(lead.xStd[0], 0.75, 6.0))
      sy = float(np.clip(lead.yStd[0], 0.25, 1.5))
      sv = float(np.clip(lead.vStd[0], 0.5, 3.0))
      score = residual_d ** 2 / (sd ** 2 + 0.25 ** 2)
      score += residual_y ** 2 / (sy ** 2 + 0.25 ** 2)
      score += residual_v ** 2 / (sv ** 2 + 0.35 ** 2)
      return score if math.isfinite(score) else math.inf
    except (AttributeError, IndexError, TypeError, ValueError):
      return math.inf

  def _match(self, tracks, lead, model_v_ego: float, preferred_id: int):
    candidates = []
    for identifier, track in tracks.items():
      if identifier not in self.evidence:
        continue
      score = self._score(track, lead, model_v_ego)
      gate = 14.156 if identifier == preferred_id and self.evidence[identifier].count >= 3 else 11.345
      if score <= gate:
        candidates.append((score, identifier, track))
    if not candidates:
      return None
    best_score, _, best = min(candidates)
    preferred = next((candidate for candidate in candidates if candidate[1] == preferred_id), None)
    return preferred[2] if preferred and preferred[0] <= best_score + 3.841 else best

  @staticmethod
  def _trailing_velocity(evidence: Evidence, model_a_rel: float) -> float | None:
    if len(evidence.distances) < 8:
      return None
    end_time = evidence.distances[-1][0]
    samples = [(t, d) for t, d in evidence.distances if t >= end_time - 0.6]
    if len(samples) < 8 or samples[-1][0] - samples[0][0] < 0.4:
      return None
    mean_t = sum(t for t, _ in samples) / len(samples)
    mean_d = sum(d for _, d in samples) / len(samples)
    xx = sum((t - mean_t) ** 2 for t, _ in samples)
    if xx <= 1e-9:
      return None
    slope = sum((t - mean_t) * (d - mean_d) for t, d in samples) / xx
    rmse = math.sqrt(sum((d - mean_d - slope * (t - mean_t)) ** 2 for t, d in samples) / len(samples))
    return slope + model_a_rel * (end_time - mean_t) if math.isfinite(slope) and rmse <= 0.15 else None

  def _model_lead(self, track, lead, v_ego: float, model_v_ego: float, model_a_ego: float,
                  model_prob: float, selected_frames: int, preferred_id: int):
    v_rel = float(lead.v[0] - model_v_ego)
    a_rel = float(lead.a[0] - model_a_ego)
    evidence = self.evidence[track.identifier]
    if (track.identifier == preferred_id and evidence.count >= 20 and selected_frames >= 12 and
        float(lead.prob) >= 0.75):
      trailing = self._trailing_velocity(evidence, a_rel)
      raw_current = track.vRel + a_rel * 0.55
      if (trailing is not None and math.isfinite(raw_current) and abs(raw_current - trailing) <= 0.75 and
          abs(trailing - v_rel) >= 0.2):
        v_rel += float(np.clip(0.1 * (trailing - v_rel), -0.1, 0.1))
    v_lead = v_ego + v_rel
    return {"dRel": float(track.dRel), "yRel": float(track.yRel), "vRel": v_rel,
            "vLead": v_lead, "vLeadK": v_lead, "aLeadK": float(lead.a[0]), "aLeadTau": 0.3,
            "modelProb": model_prob, "present": True, "radar": True, "radarTrackId": track.identifier}

  def _low_speed_sane(self, track, model, v_ego: float, previously_selected: bool) -> bool:
    evidence = self.evidence[track.identifier]
    path_x, path_y = list(model.position.x), list(model.position.y)
    if len(path_x) < 2 or len(path_x) != len(path_y) or len(evidence.distances) < 8:
      evidence.path_frames = 0
      return False
    object_x = track.dRel + RADAR_TO_CAMERA
    if not path_x[0] <= object_x <= path_x[-1]:
      evidence.path_frames = 0
      return False
    path_offset = abs(-track.yRel - float(np.interp(object_x, path_x, path_y)))
    history = list(evidence.distances)[-8:]
    span = history[-1][0] - history[0][0]
    if span <= 0 or abs((history[-1][1] - history[0][1]) / span - track.vRel) > 1.5:
      evidence.path_frames = 0
      return False
    if previously_selected:
      evidence.path_frames = 0
      return (evidence.count >= 8 and v_ego < 4.0 and 1.5 < track.dRel < 25.0 and path_offset < 1.1)
    aligned = (evidence.count >= 20 and v_ego < 3.0 and 1.5 < track.dRel < 18.0 and
               abs(track.yRel) < 1.25 and path_offset < 0.65 and -1.5 < track.vLead < 6.0)
    evidence.path_frames = evidence.path_frames + 1 if aligned else 0
    return evidence.path_frames >= 8

  def lead(self, index: int, tracks, lead, model, v_ego: float, model_v_ego: float,
           model_a_ego: float, model_prob: float, ready: bool, fresh: bool):
    from openpilot.selfdrive.controls.radard import get_RadarState_from_vision

    previous = self.selected[index]
    track = self._match(tracks, lead, model_v_ego, previous) if fresh and ready and model_prob > 0.5 else None
    if track is not None:
      result = self._model_lead(track, lead, v_ego, model_v_ego, model_a_ego,
                                model_prob, self.selected_frames[index], previous)
    elif ready and model_prob > 0.5:
      result = get_RadarState_from_vision(lead, v_ego, model_v_ego, model_prob)
    else:
      result = {"present": False}

    if index == 0 and fresh and ready:
      candidates = [track for identifier, track in tracks.items()
                    if identifier in self.evidence and self._low_speed_sane(track, model, v_ego, identifier == previous)]
      if candidates:
        closest = min(candidates, key=lambda candidate: candidate.dRel)
        if not result["present"] or closest.dRel < result["dRel"]:
          result = closest.get_RadarState(model_prob)

    selected = result.get("radarTrackId", -1)
    self.selected_frames[index] = self.selected_frames[index] + 1 if selected >= 0 and selected == previous else 1 if selected >= 0 else 0
    self.selected[index] = selected
    return result
