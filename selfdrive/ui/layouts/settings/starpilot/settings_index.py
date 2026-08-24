from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget

from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import StarPilotPanelInfo, StarPilotPanelType
from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import SettingRow, SettingSection


@dataclass(frozen=True)
class SettingsEntry:
  row: SettingRow
  label_path: str
  stable_path: str
  stable_id: str
  terms: str


def row_enabled(row: SettingRow) -> bool:
  try:
    return row.enabled() if row.enabled is not None else True
  except Exception:
    return False


def row_visible(row: SettingRow) -> bool:
  try:
    return row.visible() if row.visible is not None else True
  except Exception:
    return False


def section_visible(section: SettingSection) -> bool:
  try:
    return section.visible() if section.visible is not None else True
  except Exception:
    return False


def _row_terms(row: SettingRow, label_path: str, stable_path: str) -> str:
  parts = [
    row.id,
    row.title,
    tr(row.title),
    row.subtitle,
    tr(row.subtitle) if row.subtitle else "",
    row.disabled_label,
    row.action_text,
    label_path,
    stable_path,
  ]
  return " ".join(str(p) for p in parts if p).lower()


def _panel_label_title(panel_type: StarPilotPanelType, info: StarPilotPanelInfo) -> str:
  if info.name:
    return tr(info.name)
  return panel_type.name.replace("_", " ").title()


def _panel_stable_title(panel_type: StarPilotPanelType, info: StarPilotPanelInfo) -> str:
  if info.name:
    return str(info.name)
  return panel_type.name.replace("_", " ").title()


def _view_label_title(name: str, widget: Widget) -> str:
  header_title = getattr(widget, "_header_title", "")
  if header_title:
    return tr(header_title)
  return name.replace("_", " ").title()


def _view_stable_title(name: str, widget: Widget) -> str:
  header_title = getattr(widget, "_header_title", "")
  if header_title:
    return str(header_title)
  return name.replace("_", " ").title()


def _collect_rows_from_view(widget: Widget, label_path: str, stable_path: str, seen: set[int]) -> list[SettingsEntry]:
  entries: list[SettingsEntry] = []

  sections = getattr(widget, "_sections", None)
  if sections:
    for section in sections:
      if not isinstance(section, SettingSection) or not section_visible(section):
        continue
      section_label_path = label_path
      section_stable_path = stable_path
      if section.title:
        section_label_path = f"{label_path} / {tr(section.title)}"
        section_stable_path = f"{stable_path} / {section.title}"
      for row in section.rows:
        if not isinstance(row, SettingRow) or id(row) in seen:
          continue
        if row.type not in ("toggle", "value", "action"):
          continue
        if row.get_state is None and row.on_click is None and row.set_state is None and row.type != "action":
          continue
        seen.add(id(row))
        stable_id = f"{section_stable_path}::{row.id}"
        entries.append(SettingsEntry(row, section_label_path, section_stable_path, stable_id,
                                     _row_terms(row, section_label_path, section_stable_path)))

  for sub_name, sub_widget in getattr(widget, "_sub_panels", {}).items():
    if sub_widget is None:
      continue
    sub_label_path = f"{label_path} / {_view_label_title(str(sub_name), sub_widget)}"
    sub_stable_path = f"{stable_path} / {_view_stable_title(str(sub_name), sub_widget)}"
    entries.extend(_collect_rows_from_view(sub_widget, sub_label_path, sub_stable_path, seen))

  return entries


def build_settings_index(panel_provider: Callable[[], dict[StarPilotPanelType, StarPilotPanelInfo]],
                         exclude_panel_types: set[StarPilotPanelType] | None = None) -> list[SettingsEntry]:
  excluded = exclude_panel_types or {StarPilotPanelType.MAIN}
  seen: set[int] = set()
  entries: list[SettingsEntry] = []

  for panel_type, info in panel_provider().items():
    if panel_type in excluded or info.instance is None:
      continue
    label_path = _panel_label_title(panel_type, info)
    stable_path = _panel_stable_title(panel_type, info)
    entries.extend(_collect_rows_from_view(info.instance, label_path, stable_path, seen))

  return entries


def filter_settings_entries(entries: list[SettingsEntry], query: str, max_results: int) -> list[SettingsEntry]:
  query = query.strip().lower()
  if len(query) < 2:
    return []

  tokens = [token for token in query.split() if token]
  scored: list[tuple[int, SettingsEntry]] = []
  for entry in entries:
    if not all(token in entry.terms for token in tokens):
      continue
    title = str(entry.row.title).lower()
    translated_title = tr(entry.row.title).lower()
    row_id = entry.row.id.lower()
    score = 0
    if title.startswith(query) or translated_title.startswith(query) or row_id.startswith(query):
      score += 40
    if query in title or query in translated_title:
      score += 25
    if query in row_id:
      score += 20
    if row_visible(entry.row):
      score += 5
    scored.append((score, entry))

  scored.sort(key=lambda item: (-item[0], tr(item[1].row.title).lower(), item[1].label_path))
  return [entry for _score, entry in scored[:max_results]]
