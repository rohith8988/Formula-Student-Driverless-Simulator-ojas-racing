"""Formula Student Simulator TUI.

A Textual-based terminal control panel for the FS simulator.

Run with:
    ros2 run simulator fs_tui
  or directly:
    python -m fs_tui.panel

Key bindings
------------
  l         Launch (generate + start Gazebo)
  s         Stop all (kill Gazebo + ROS nodes)
  ctrl+r    Reset car to start position
  q         Quit (does NOT stop the sim)
"""

from __future__ import annotations

import inspect
import sys
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

from textual import on
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, Vertical
from textual.reactive import reactive
from textual.widgets import (
    Button,
    Footer,
    Header,
    Label,
    RadioButton,
    RadioSet,
    RichLog,
    Static,
)

from fs_tui.sim_process import SimController, SimulationConfig

# ── helpers ───────────────────────────────────────────────────────────────────


def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def _find_rviz_config() -> str | None:
    """Return the fs_vehicle RViz config path, or None if not found."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("fs_vehicle")
        p = Path(share) / "config" / "fs_car.rviz"
        if p.exists():
            return str(p)
    except Exception:
        pass
    # source-tree fallback
    here = Path(__file__).resolve()
    for parent in here.parents:
        p = parent / "fs_vehicle" / "config" / "fs_car.rviz"
        if p.exists():
            return str(p)
    return None


TRACK_TYPE_BY_RADIO_ID = {
    "rb-trackdrive":      "trackdrive",
    "rb-test-trackdrive": "test_trackdrive",
    "rb-skidpad":         "skidpad",
    "rb-accel":           "acceleration",
}

RENDER_MODE_BY_RADIO_ID = {
    "rb-render-high":        "high",
    "rb-render-performance": "performance",
}


# ── CSS ───────────────────────────────────────────────────────────────────────

CSS = """
Screen {
    background: $surface;
}

#main {
    height: 1fr;
}

#config-panel {
    width: 38;
    border: solid $primary;
    padding: 1 2;
    margin: 1 1;
}

#right-panel {
    width: 1fr;
    margin: 1 1;
}

.panel-title {
    text-style: bold;
    color: $accent;
    margin-bottom: 1;
}

.section-label {
    color: $text-muted;
    margin-top: 1;
}

#tools-group Button {
    width: 1fr;
    margin-bottom: 1;
}

#status-box {
    border: solid $primary;
    padding: 1 2;
    height: 9;
    margin-bottom: 1;
}

#action-bar {
    height: 3;
    margin-bottom: 1;
}

#action-bar Button {
    margin-right: 1;
}

#log-box {
    border: solid $primary;
    height: 1fr;
    padding: 0 1;
}

RichLog {
    height: 1fr;
}
"""


# ── status widget ─────────────────────────────────────────────────────────────


class StatusBox(Static):
    """Live status panel."""

    def __init__(self) -> None:
        super().__init__("", id="status-box")
        self._sim_state = "stopped"
        self._track_info = "—"
        self._world_name = "—"
        self._elapsed_s = 0
        self._start_time: datetime | None = None

    def on_mount(self) -> None:
        self.set_interval(1.0, self._tick)

    def _tick(self) -> None:
        if self._start_time and self._sim_state == "running":
            self._elapsed_s = int((datetime.now() - self._start_time).total_seconds())
        self._refresh()

    def set_running(self, track_info: str, world_name: str) -> None:
        self._sim_state = "running"
        self._track_info = track_info
        self._world_name = world_name
        self._start_time = datetime.now()
        self._elapsed_s = 0
        self._refresh()

    def set_stopped(self) -> None:
        self._sim_state = "stopped"
        self._start_time = None
        self._world_name = "—"
        self._refresh()

    def _refresh(self) -> None:
        state_str = (
            "[green]running[/green]"
            if self._sim_state == "running"
            else "[red]stopped[/red]"
        )
        mins, secs = divmod(self._elapsed_s, 60)
        self.update(
            f"[bold]Status[/bold]\n"
            f"  Simulation : {state_str}\n"
            f"  Track      : {self._track_info}\n"
            f"  Elapsed    : {mins:02d}:{secs:02d}\n"
            f"  World      : {self._world_name}"
        )


# ── main app ──────────────────────────────────────────────────────────────────


class FsSimApp(App[None]):
    """Formula Student Simulator Control Panel."""

    CSS = CSS
    TITLE = "FS Simulator"
    SUB_TITLE = "Formula Student Driverless"

    BINDINGS = [
        Binding("l", "launch", "Launch"),
        Binding("s", "stop", "Stop"),
        Binding("ctrl+r", "reset_car", "Reset Car"),
        Binding("q", "quit", "Quit"),
    ]

    track_type: reactive[str] = reactive("trackdrive")
    render_mode: reactive[str] = reactive("high")
    sim_running: reactive[bool] = reactive(False)

    def __init__(self) -> None:
        super().__init__()
        self._custom_map_file: str | None = None
        self._map_files: list[Path] = self._load_map_files()
        self._controller = SimController(
            log=self._log,
            schedule_timer=self._schedule_timer,
            on_launch_started=self._set_running_state,
            on_track_ready=self._set_running_status,
            on_stopped=self._set_stopped_state,
        )

    @staticmethod
    def _load_map_files() -> list[Path]:
        try:
            from fs_track_generator.artifacts import list_map_files

            return list_map_files()
        except Exception:
            return []

    # ── layout ───────────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        with Horizontal(id="main"):
            with Vertical(id="config-panel"):
                yield Static("Track Configuration", classes="panel-title")

                yield Label("Track Type", classes="section-label")
                with RadioSet(id="track-type-radio"):
                    yield RadioButton("Trackdrive  (~450 m)", value=True, id="rb-trackdrive")
                    yield RadioButton("Test Track  (~300 m)", id="rb-test-trackdrive")
                    yield RadioButton("Skidpad    (figure-8)", id="rb-skidpad")
                    yield RadioButton("Acceleration (75 m)", id="rb-accel")
                    for i, path in enumerate(self._map_files):
                        yield RadioButton(f"Map: {path.name}", id=f"rb-map-{i}")

                yield Label("Render", classes="section-label")
                with RadioSet(id="render-radio"):
                    yield RadioButton("High", value=True, id="rb-render-high")
                    yield RadioButton("Performance", id="rb-render-performance")

                yield Label("Tools", classes="section-label")
                with Vertical(id="tools-group"):
                    yield Button("RViz2", id="btn-rviz", variant="default")
                    yield Button("PS Teleop", id="btn-ps-teleop", variant="default")

            with Vertical(id="right-panel"):
                yield StatusBox()

                with Horizontal(id="action-bar"):
                    yield Button("▶  Launch", id="btn-launch", variant="success")
                    yield Button("■  Stop All", id="btn-stop", variant="error", disabled=True)
                    yield Button("↺  Reset Car", id="btn-reset", variant="warning", disabled=True)

                with Container(id="log-box"):
                    yield Static("Log", classes="panel-title")
                    yield RichLog(id="log", highlight=True, markup=True)

        yield Footer()

    async def on_unmount(self) -> None:
        """Close subprocess transports before the event loop shuts down."""
        await self._controller.close()

    # ── event handlers ───────────────────────────────────────────────

    @on(RadioSet.Changed)
    def _on_radio_changed(self, event: RadioSet.Changed) -> None:
        radio_set_id = str(event.radio_set.id)
        pressed_id = str(event.pressed.id)

        if radio_set_id == "render-radio":
            self.render_mode = RENDER_MODE_BY_RADIO_ID.get(pressed_id, "high")
        elif radio_set_id == "track-type-radio":
            if pressed_id in TRACK_TYPE_BY_RADIO_ID:
                self.track_type = TRACK_TYPE_BY_RADIO_ID[pressed_id]
                self._custom_map_file = None
            elif pressed_id.startswith("rb-map-"):
                idx = int(pressed_id[len("rb-map-"):])
                self._custom_map_file = str(self._map_files[idx])

    @on(Button.Pressed)
    async def _on_button_pressed(self, event: Button.Pressed) -> None:
        action_map = {
            "btn-launch":    "launch",
            "btn-stop":      "stop",
            "btn-reset":     "reset_car",
            "btn-rviz":      "launch_rviz",
            "btn-ps-teleop": "launch_ps_teleop",
        }
        action_name = action_map.get(str(event.button.id))
        if action_name is None:
            return
        await self._invoke_action(action_name)

    # ── actions ──────────────────────────────────────────────────────

    async def action_launch(self) -> None:
        await self._controller.launch(
            SimulationConfig(
                track_type=self.track_type,
                render_mode=self.render_mode,
                map_file=self._custom_map_file,
            )
        )

    async def action_stop(self) -> None:
        await self._controller.stop()

    async def action_reset_car(self) -> None:
        await self._controller.reset_car()

    async def action_launch_rviz(self) -> None:
        await self._controller.launch_rviz(_find_rviz_config())

    async def action_launch_ps_teleop(self) -> None:
        await self._controller.launch_ps_teleop()

    # ── private helpers ───────────────────────────────────────────────

    async def _invoke_action(self, action_name: str) -> None:
        action = getattr(self, f"action_{action_name}", None)
        if action is None:
            return
        result = action()
        if inspect.isawaitable(result):
            await result

    def _schedule_timer(self, delay_s: float, callback: Callable[[], None]) -> None:
        self.set_timer(delay_s, callback)

    def _set_running_state(self) -> None:
        self.sim_running = True
        self._update_buttons()

    def _set_running_status(self, track_info: str, world_name: str) -> None:
        self.query_one(StatusBox).set_running(track_info, world_name)

    def _set_stopped_state(self) -> None:
        self.sim_running = False
        self._update_buttons()
        self.query_one(StatusBox).set_stopped()

    def _log(self, msg: str) -> None:
        log = self.query_one("#log", RichLog)
        log.write(f"[dim]{_ts()}[/dim]  {msg}")

    def _update_buttons(self) -> None:
        self.query_one("#btn-launch", Button).disabled = self.sim_running
        self.query_one("#btn-stop", Button).disabled = not self.sim_running
        self.query_one("#btn-reset", Button).disabled = not self.sim_running


def _suppress_loop_closed_noise() -> None:
    """Filter out the 'Event loop is closed' ResourceWarning from asyncio subprocess
    transports whose __del__ fires after the event loop has shut down.  This is a
    known Python/asyncio limitation on exit — no data is lost and the processes
    themselves are unaffected.  Everything else is passed to the default handler.
    """
    _orig = sys.unraisablehook

    def _hook(args: sys.UnraisableHookArgs) -> None:
        if args.exc_type is RuntimeError and str(args.exc_value) == "Event loop is closed":
            return
        _orig(args)

    sys.unraisablehook = _hook


def main() -> None:
    _suppress_loop_closed_noise()
    app = FsSimApp()
    app.run()


if __name__ == "__main__":
    main()
