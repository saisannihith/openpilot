from __future__ import annotations
from collections.abc import Callable
import pyray as rl

from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.application import FontWeight, MousePos, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.label import gui_label

from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import StarPilotPanelType, StarPilotPanelInfo, FrameCachedParams
from openpilot.selfdrive.ui.layouts.settings.starpilot.sounds import StarPilotSoundsLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.driving_model import StarPilotDrivingModelLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.longitudinal import StarPilotLongitudinalLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.lateral import StarPilotLateralLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.maps import StarPilotMapsLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.navigation import StarPilotNavigationLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.system_settings import StarPilotSystemLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.appearance import StarPilotAppearanceLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.vehicle import StarPilotVehicleSettingsLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.quick_controls import StarPilotQuickControlsLayout
from openpilot.selfdrive.ui.layouts.settings.starpilot.search import StarPilotSearchLayout

from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import TileGrid, HubTile, SPACING, BreadcrumbController, AETHER_LIST_METRICS, AetherListColors, draw_hud_background, draw_text_fit_common

class StarPilotLayout(Widget):
  CATEGORIES = [
    {
      "title": "Quick Controls",
      "icon": "system",
      "panel": "QUICK_CONTROLS",
    },
    {
      "title": "Search Settings",
      "icon": "display",
      "panel": "SEARCH",
    },
    {
      "title": "Sounds & Alerts",
      "icon": "sound",
      "panel": "SOUNDS",
    },
    {
      "title": "Driving Model",
      "icon": "aicar",
      "panel": "DRIVING_MODEL",
    },
    {
      "title": "Driving Controls",
      "icon": "steering",
      "children": [
        {
          "title": "Navigation & Maps",
          "icon": "navigate",
          "children": [
            {"title": "Map Data", "panel": "MAPS", "icon": "navigate"},
            {"title": "Navigation", "panel": "NAVIGATION", "icon": "road"},
          ],
        },
        {"title": "Gas / Brake", "panel": "LONGITUDINAL", "icon": "road"},
        {"title": "Steering", "panel": "LATERAL", "icon": "steering"},
      ],
    },
    {
      "title": "System",
      "icon": "system",
      "panel": "SYSTEM",
    },
    {
      "title": "Appearance",
      "icon": "display",
      "panel": "VISUALS",
    },
    {
      "title": "Vehicle Settings",
      "icon": "vehicle",
      "panel": "VEHICLE",
    },
  ]

  PANEL_TYPE_MAP = {
    "QUICK_CONTROLS": StarPilotPanelType.QUICK_CONTROLS,
    "SEARCH": StarPilotPanelType.SEARCH,
    "SOUNDS": StarPilotPanelType.SOUNDS,
    "SYSTEM": StarPilotPanelType.SYSTEM,
    "DRIVING_MODEL": StarPilotPanelType.DRIVING_MODEL,
    "LONGITUDINAL": StarPilotPanelType.LONGITUDINAL,
    "LATERAL": StarPilotPanelType.LATERAL,
    "MAPS": StarPilotPanelType.MAPS,
    "NAVIGATION": StarPilotPanelType.NAVIGATION,
    "VISUALS": StarPilotPanelType.VISUALS,
    "VEHICLE": StarPilotPanelType.VEHICLE,
  }

  def __init__(self):
    super().__init__()
    self._params = FrameCachedParams()

    self._current_panel = StarPilotPanelType.MAIN
    self._hub_path: list[dict] = []
    self._selected_leaf: dict | None = None
    # Kept as a compatibility alias for callers that only need the top-level
    # folder index.  Nested hub navigation is represented by _hub_path.
    self._current_category_idx: int | None = None
    self._depth_callback: Callable | None = None
    self._settings_layout = None

    StarPilotLayout.active_instance = self

    self._panel_stack: list[tuple[StarPilotPanelType, str]] = []  
    self._sub_panel_callbacks: dict[str, Callable] = {}  

    self._panels = {
      StarPilotPanelType.MAIN: StarPilotPanelInfo("", None),
      StarPilotPanelType.QUICK_CONTROLS: StarPilotPanelInfo(tr_noop("Quick Controls"), StarPilotQuickControlsLayout(lambda: getattr(self, "_panels", {}))),
      StarPilotPanelType.SEARCH: StarPilotPanelInfo(tr_noop("Search Settings"), StarPilotSearchLayout(lambda: getattr(self, "_panels", {}))),
      StarPilotPanelType.SOUNDS: StarPilotPanelInfo(tr_noop("Sounds"), StarPilotSoundsLayout()),
      StarPilotPanelType.SYSTEM: StarPilotPanelInfo(tr_noop("System Settings"), StarPilotSystemLayout()),
      StarPilotPanelType.DRIVING_MODEL: StarPilotPanelInfo(tr_noop("Driving Model"), StarPilotDrivingModelLayout()),
      StarPilotPanelType.LONGITUDINAL: StarPilotPanelInfo(tr_noop("Gas / Brake"), StarPilotLongitudinalLayout()),
      StarPilotPanelType.LATERAL: StarPilotPanelInfo(tr_noop("Steering"), StarPilotLateralLayout()),
      StarPilotPanelType.MAPS: StarPilotPanelInfo(tr_noop("Map Data"), StarPilotMapsLayout()),
      StarPilotPanelType.NAVIGATION: StarPilotPanelInfo(tr_noop("Navigation"), StarPilotNavigationLayout()),
      StarPilotPanelType.VISUALS: StarPilotPanelInfo(tr_noop("Appearance"), StarPilotAppearanceLayout()),
      StarPilotPanelType.VEHICLE: StarPilotPanelInfo(tr_noop("Vehicle Settings"), StarPilotVehicleSettingsLayout()),
    }

    self._setup_sub_panels(
      StarPilotPanelType.LONGITUDINAL,
      StarPilotPanelType.QUICK_CONTROLS,
      StarPilotPanelType.SEARCH,
      StarPilotPanelType.SOUNDS,
      StarPilotPanelType.SYSTEM,
      StarPilotPanelType.LATERAL,
      StarPilotPanelType.MAPS,
      StarPilotPanelType.NAVIGATION,
      StarPilotPanelType.VISUALS,
      StarPilotPanelType.VEHICLE,
    )

    self._breadcrumbs = BreadcrumbController()
    self._main_grid = TileGrid(columns=None, padding=8)
    self._rebuild_grid()

  def set_depth_callback(self, callback: Callable):
    self._depth_callback = callback

  def set_settings_layout(self, settings_layout):
    self._settings_layout = settings_layout

  @property
  def hub_path(self) -> tuple[dict, ...]:
    return tuple(self._hub_path)

  def navigate_back(self):
    if self._panel_stack:
      self._panel_stack.pop()
      self._commit_navigation()
    elif self._current_panel != StarPilotPanelType.MAIN:
      # A panel always returns to the folder that launched it.
      self._set_current_panel(StarPilotPanelType.MAIN)
    elif self._hub_path:
      # Once the grid is visible, each back step removes one hub folder.
      self._hub_path.pop()
      self._selected_leaf = None
      self._sync_legacy_category_idx()
      self._rebuild_grid()
      self._commit_navigation()

  def reset_to_root(self):
    """Close nested content and restore the primary six-tile hub."""
    self._hub_path.clear()
    self._selected_leaf = None
    self._sync_legacy_category_idx()
    self._set_current_panel(StarPilotPanelType.MAIN)

  def navigate_to_hub_depth(self, depth: int):
    """Jump to a folder in the current hub path from a breadcrumb."""
    depth = max(0, min(depth, len(self._hub_path)))
    self._hub_path = self._hub_path[:depth]
    self._selected_leaf = None
    self._sync_legacy_category_idx()
    self._set_current_panel(StarPilotPanelType.MAIN)

  def _sync_legacy_category_idx(self):
    if self._hub_path:
      self._current_category_idx = self.CATEGORIES.index(self._hub_path[0])
    else:
      self._current_category_idx = None

  def _open_folder(self, folder: dict):
    if "children" not in folder:
      return
    self._hub_path.append(folder)
    self._selected_leaf = None
    self._sync_legacy_category_idx()
    self._set_current_panel(StarPilotPanelType.MAIN)

  def _open_leaf(self, leaf: dict):
    panel_key = leaf.get("panel")
    if panel_key is None:
      return
    self._selected_leaf = leaf
    self._set_current_panel(self.PANEL_TYPE_MAP[panel_key])

  def _update_depth(self):
    # Root = 0, each visible hub folder = 1, and an open backend panel adds
    # one more level.  Existing panel sub-pages remain below that panel.
    depth = len(self._hub_path)
    if self._current_panel != StarPilotPanelType.MAIN:
      depth += 1
    depth += len(self._panel_stack)

    if self._depth_callback:
      self._depth_callback(depth)

  def _commit_navigation(self):
    self._update_sub_panel_visibility()
    self._update_depth()

  def _push_sub_panel(self, sub_panel_name: str):
    if sub_panel_name:
      self._panel_stack.append((self._current_panel, sub_panel_name))
    else:
      while self._panel_stack and self._panel_stack[-1][0] == self._current_panel:
        self._panel_stack.pop()
    self._commit_navigation()

  def _update_sub_panel_visibility(self):
    panel = self._panels[self._current_panel].instance
    current_sub = self._get_current_sub_panel()
    if panel and hasattr(panel, 'set_current_sub_panel'):
      panel.set_current_sub_panel(current_sub)

  def _get_current_sub_panel(self) -> str:
    if self._panel_stack and self._panel_stack[-1][0] == self._current_panel:
      return self._panel_stack[-1][1]
    return ""

  def _setup_sub_panels(self, *panel_types: StarPilotPanelType):
    for panel_type in panel_types:
      panel = self._panels[panel_type].instance
      if panel and hasattr(panel, 'set_navigate_callback'):
        panel.set_navigate_callback(self._push_sub_panel)

  def _rebuild_grid(self):
    state = tuple(id(folder) for folder in self._hub_path)
    if getattr(self, "_last_grid_state", None) == state:
      return
    self._last_grid_state = state
    self._main_grid.clear()

    visible_nodes = self.CATEGORIES if not self._hub_path else self._hub_path[-1]["children"]
    for node in visible_nodes:
      def on_click(item=node):
        if "children" in item:
          self._open_folder(item)
        else:
          self._open_leaf(item)

      tile = HubTile(
        title=tr(node["title"]),
        desc=tr(node.get("desc", "")),
        icon_key=node["icon"],
        on_click=on_click,
        bg_color=node.get("color")
      )
      self._main_grid.add_tile(tile)

  def _set_current_panel(self, panel_type: StarPilotPanelType):
    if panel_type != self._current_panel:

      if self._current_panel != StarPilotPanelType.MAIN:
        old = self._panels[self._current_panel].instance
        old.hide_event()
        if hasattr(old, 'set_current_sub_panel'):
          old.set_current_sub_panel("")
      self._current_panel = panel_type
      self._panel_stack.clear()
      if panel_type != StarPilotPanelType.MAIN:
        self._panels[panel_type].instance.show_event()
      else:
        self._selected_leaf = None
        self._rebuild_grid()
    elif panel_type == StarPilotPanelType.MAIN:
      self._selected_leaf = None
      self._rebuild_grid()
      self._panel_stack.clear()

    self._commit_navigation()

  def _render(self, rect: rl.Rectangle):
    TOP_BAR_HEIGHT = 58
    BOTTOM_BAR_HEIGHT = 2
    content_rect = rl.Rectangle(rect.x, rect.y + TOP_BAR_HEIGHT, rect.width, rect.height - TOP_BAR_HEIGHT - BOTTOM_BAR_HEIGHT)

    shell_margin = 4.0
    shell_w = min(rect.width - shell_margin * 2, AETHER_LIST_METRICS.max_content_width)
    shell_x = rect.x + (rect.width - shell_w) / 2

    # 0. Draw top bar with HubTile-style purple glow
    glass_rect = rl.Rectangle(shell_x, rect.y + 2, shell_w, TOP_BAR_HEIGHT - 4)
    draw_hud_background(glass_rect, AetherListColors.PRIMARY, radius_px=26)

    accessory_width = min(self._header_accessory_width(), max(0.0, glass_rect.width - 360.0))

    # 1. Draw breadcrumbs in top bar
    crumb_rect = rl.Rectangle(glass_rect.x, glass_rect.y, glass_rect.width - accessory_width, glass_rect.height)
    self._breadcrumbs.draw(crumb_rect)
    self._draw_header_accessory(rl.Rectangle(glass_rect.x + glass_rect.width - accessory_width, glass_rect.y,
                                             accessory_width, glass_rect.height))

    # 4. Render active content panel
    if self._current_panel == StarPilotPanelType.MAIN:
      grid_rect = rl.Rectangle(shell_x, content_rect.y + 4, shell_w, content_rect.height - 8)
      self._main_grid.render(grid_rect)
    else:
      panel = self._panels[self._current_panel]
      if panel.instance:
        panel.instance.render(content_rect)

  def _header_accessory_width(self) -> float:
    if self._current_panel == StarPilotPanelType.DRIVING_MODEL:
      return 680.0
    return 0.0

  def _draw_header_accessory(self, rect: rl.Rectangle):
    if self._current_panel != StarPilotPanelType.DRIVING_MODEL or rect.width <= 0:
      return

    panel = self._panels[self._current_panel].instance
    current_entry = panel.current_entry() if hasattr(panel, "current_entry") else None
    if current_entry is None:
      return

    prefix = tr("Current") + ":"
    font_size = 32
    prefix_font = gui_app.font(FontWeight.MEDIUM)
    model_font = gui_app.font(FontWeight.SEMI_BOLD)
    prefix_w = measure_text_cached(prefix_font, prefix, font_size).x
    model_w = min(measure_text_cached(model_font, current_entry.name, font_size).x, max(0.0, rect.width - prefix_w - 34))
    right_pad = 22.0
    gap = 10.0
    model_rect = rl.Rectangle(rect.x + rect.width - right_pad - model_w, rect.y, model_w, rect.height)
    label_rect = rl.Rectangle(model_rect.x - gap - prefix_w, rect.y, prefix_w, rect.height)

    gui_label(label_rect, prefix, font_size, AetherListColors.SUBTEXT, FontWeight.MEDIUM,
              alignment=rl.GuiTextAlignment.TEXT_ALIGN_RIGHT)
    draw_text_fit_common(
      model_font,
      current_entry.name,
      rl.Vector2(model_rect.x, model_rect.y + (model_rect.height - font_size) / 2),
      model_rect.width,
      font_size,
      color=AetherListColors.SUCCESS,
    )

  def _handle_mouse_press(self, mouse_pos: MousePos):
    self._breadcrumbs.init_interaction(mouse_pos)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    action = self._breadcrumbs.finish_interaction(mouse_pos)
    if action:
      self._breadcrumbs.handle_click(action)

  def _handle_mouse_event(self, mouse_event):
    self._breadcrumbs.update_interaction(mouse_event.pos)

  def show_event(self):
    super().show_event()
    self._breadcrumbs.cancel_interaction()
    if self._current_panel != StarPilotPanelType.MAIN:
      self._panels[self._current_panel].instance.show_event()

  def hide_event(self):
    super().hide_event()
    self._breadcrumbs.cancel_interaction()
    if self._current_panel != StarPilotPanelType.MAIN:
      self._panels[self._current_panel].instance.hide_event()
