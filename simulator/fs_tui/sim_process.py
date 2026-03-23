"""Simulator process control for the Formula Student TUI."""

from __future__ import annotations

import asyncio
import math
import os
import shutil
import signal
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

DEFAULT_WORLD_NAME = "fs_world"
DEFAULT_MODEL_NAME = "fs_car"

GAZEBO_KILL_PATTERNS = (
    r"ros2 launch simulator sim.launch.py",
    r"ros2 launch simulator view_generated_track.launch.py",
    r"ros2 launch fs_vehicle spawn_vehicle.launch.py",
    r"spawn_fs_car",
    r"gz sim",
    r"gz_sim_vendor",
    r"ruby.*gz",
)

START_POSE_LOAD_DELAY_S = 5.0
METADATA_RETRY_DELAY_S = 3.0
PKILL_TIMEOUT_S = 2.0
TERMINATE_TIMEOUT_S = 6.0
RESET_CMD_VEL_TIMEOUT_S = 3.0
RESET_SERVICE_TIMEOUT_S = 5.0
PROCESS_GRACE_DELAY_S = 0.5



@dataclass(frozen=True, slots=True)
class SimulationConfig:
    """User-selected simulator launch settings."""

    track_type: str
    render_mode: str
    map_file: str | None = None  # custom SDF path; bypasses track generation when set


def _read_latest_meta() -> dict | None:
    """Return the metadata dict for the most recently generated track, or None."""

    try:
        from fs_track_generator.artifacts import latest_generated_world, read_structured_file

        paths = latest_generated_world()
        return read_structured_file(paths["meta"])
    except Exception:
        return None


# Maps metadata keys → the short names used in artifact path dicts.
_ARTIFACT_KEY_MAP = {
    "world_file":  "world",
    "cone_file":   "cones",
    "bridge_file": "bridge",
    "meta_file":   "meta",
}


def _artifact_paths_from_meta(meta: dict) -> dict[str, Path]:
    """Return the generated artifact paths recorded in metadata."""
    return {
        path_key: Path(meta[meta_key])
        for meta_key, path_key in _ARTIFACT_KEY_MAP.items()
        if meta.get(meta_key)
    }


def _reset_car_cmd(start_pose: dict, world_name: str, model_name: str) -> list[str]:
    """Build the gz service command to teleport the car to start_pose."""

    x = start_pose["x"]
    y = start_pose["y"]
    z = start_pose["z"]
    yaw = start_pose["yaw"]
    w = math.cos(yaw / 2.0)
    z_rot = math.sin(yaw / 2.0)
    req = (
        f'name: "{model_name}" '
        f"position: {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} "
        f"orientation: {{w: {w:.6f} x: 0.0 y: 0.0 z: {z_rot:.6f}}}"
    )
    return [
        "gz",
        "service",
        "-s",
        f"/world/{world_name}/set_pose",
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        req,
    ]


class SimController:
    """Owns simulator subprocesses, metadata, and cleanup."""

    def __init__(
        self,
        *,
        log: Callable[[str], None],
        schedule_timer: Callable[[float, Callable[[], None]], None],
        on_launch_started: Callable[[], None],
        on_track_ready: Callable[[str, str], None],
        on_stopped: Callable[[], None],
    ) -> None:
        self._log = log
        self._schedule_timer = schedule_timer
        self._on_launch_started = on_launch_started
        self._on_track_ready = on_track_ready
        self._on_stopped = on_stopped

        self.sim_running = False
        self._launch_proc: asyncio.subprocess.Process | None = None
        self._reader_task: asyncio.Task[None] | None = None
        self._watcher_task: asyncio.Task[None] | None = None
        self._start_pose: dict | None = None
        self._current_meta: dict | None = None
        self._tool_procs: dict[str, asyncio.subprocess.Process] = {}

    async def close(self) -> None:
        """Cancel background tasks and close the stdout pipe without killing the process.

        Called when the TUI exits so the asyncio transport is released before
        the event loop closes (prevents 'Event loop is closed' noise on exit).
        """
        for task in (self._reader_task, self._watcher_task):
            if task and not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._reader_task = None
        self._watcher_task = None

        if self._launch_proc is not None:
            stdout = self._launch_proc.stdout
            if stdout is not None:
                stdout.feed_eof()
                transport = getattr(stdout, "_transport", None)
                if transport is not None:
                    transport.close()
            self._launch_proc = None

        # Terminate tool subprocesses (RViz, teleoperation, etc.)
        for proc in self._tool_procs.values():
            if proc.returncode is None:
                try:
                    proc.terminate()
                except ProcessLookupError:
                    pass
        self._tool_procs.clear()

    async def launch(self, config: SimulationConfig) -> None:
        """Launch the simulator with the requested configuration."""

        if self.sim_running:
            self._log("[yellow]Stop the current simulation before launching a new one.[/yellow]")
            return

        self._cleanup_generated_track(_read_latest_meta())
        await self._kill_stray_gazebo()
        self._reset_runtime_state()

        if config.map_file:
            self._log(
                f"[bold]Launching[/bold]  map=[cyan]{Path(config.map_file).name}[/cyan]  "
                f"render=[cyan]{config.render_mode}[/cyan]"
            )
        else:
            self._log(
                f"[bold]Launching[/bold]  type=[cyan]{config.track_type}[/cyan]  "
                f"render=[cyan]{config.render_mode}[/cyan]"
            )

        _gamemoderun = shutil.which("gamemoderun") or "/usr/games/gamemoderun"
        _use_gamemode = Path(_gamemoderun).exists()
        cmd = (
            [_gamemoderun] if _use_gamemode else []
        ) + [
            "ros2",
            "launch",
            "simulator",
            "sim.launch.py",
            f"track_type:={config.track_type}",
            f"render_mode:={config.render_mode}",
        ]
        if config.map_file:
            cmd.append(f"map_file:={config.map_file}")

        try:
            self._launch_proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,
            )
        except FileNotFoundError:
            self._log("[red]ros2 not found — source your ROS2 workspace first.[/red]")
            return

        self.sim_running = True
        self._on_launch_started()

        loop = asyncio.get_running_loop()
        self._reader_task = loop.create_task(self._read_output(self._launch_proc))
        self._watcher_task = loop.create_task(self._watch_process(self._launch_proc))
        self._schedule_timer(START_POSE_LOAD_DELAY_S, self._load_start_pose)

    async def stop(self) -> None:
        """Stop the simulator, close RViz2, and clean up temporary files."""

        if not self.sim_running and self._launch_proc is None:
            await self._kill_stray_gazebo()
            self._cleanup_generated_track(_read_latest_meta())
            self._kill_tool("rviz")
            return

        self._log("[yellow]Stopping simulation…[/yellow]")
        await self._terminate_launch_proc()
        self._kill_tool("rviz")
        self._cleanup_generated_track(self._current_meta)
        self._set_stopped_state()
        self._log("Simulation stopped.")

    def _kill_tool(self, tool_name: str) -> None:
        """Terminate a tracked tool process if it is still running."""
        proc = self._tool_procs.pop(tool_name, None)
        if proc is not None and proc.returncode is None:
            try:
                proc.terminate()
            except ProcessLookupError:
                pass

    async def launch_rviz(self, rviz_config: str | None = None) -> None:
        """Launch RViz2, optionally with a specific config file."""
        cmd = ["ros2", "run", "rviz2", "rviz2"]
        if rviz_config:
            cmd += ["-d", rviz_config]
        await self._launch_tool("rviz", cmd)

    async def launch_ps_teleop(self) -> None:
        """Launch the PlayStation controller teleop node."""
        await self._launch_tool(
            "ps_teleop",
            ["ros2", "launch", "fs_vehicle", "ps_teleop.launch.py"],
        )

    async def _launch_tool(self, tool_name: str, cmd: list[str]) -> None:
        """Start (or restart) a tool subprocess tracked by name."""
        existing = self._tool_procs.pop(tool_name, None)
        if existing is not None and existing.returncode is None:
            try:
                existing.terminate()
            except ProcessLookupError:
                pass

        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
                start_new_session=True,
            )
            self._tool_procs[tool_name] = proc
            self._log(f"[green]{tool_name} launched (pid {proc.pid}).[/green]")
        except FileNotFoundError:
            self._log(f"[red]{cmd[0]} not found — is it installed and sourced?[/red]")

    async def reset_car(self) -> None:
        """Teleport fs_car back to the saved start pose via Gazebo service."""

        if not self.sim_running:
            self._log("[yellow]Simulation is not running.[/yellow]")
            return

        if self._start_pose is None:
            self._load_start_pose()
            if self._start_pose is None:
                self._log("[red]Start pose not available yet — try again in a moment.[/red]")
                return

        self._log(
            f"Resetting car →  "
            f"x={self._start_pose['x']:.2f}  "
            f"y={self._start_pose['y']:.2f}  "
            f"yaw={self._start_pose['yaw']:.3f} rad"
        )
        meta = self._current_meta or {}
        world_name = meta.get("world_name", DEFAULT_WORLD_NAME)
        model_name = meta.get("model_name", DEFAULT_MODEL_NAME)
        asyncio.get_running_loop().create_task(
            self._do_reset(_reset_car_cmd(self._start_pose, world_name, model_name))
        )

    async def _kill_pattern(self, signal_name: str, pattern: str) -> None:
        """Best-effort pkill wrapper."""

        try:
            proc = await asyncio.create_subprocess_exec(
                "pkill",
                signal_name,
                "-f",
                pattern,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
            await asyncio.wait_for(proc.wait(), timeout=PKILL_TIMEOUT_S)
        except Exception:
            pass

    async def _kill_stray_gazebo(self) -> None:
        """Kill simulator launch and Gazebo processes left over from a previous session."""

        for pattern in GAZEBO_KILL_PATTERNS:
            await self._kill_pattern("-TERM", pattern)

        await asyncio.sleep(PROCESS_GRACE_DELAY_S)

        for pattern in GAZEBO_KILL_PATTERNS:
            await self._kill_pattern("-KILL", pattern)

        await asyncio.sleep(PROCESS_GRACE_DELAY_S)

    async def _terminate_launch_proc(self) -> None:
        """Cancel background tasks and kill the ros2 launch process group."""

        # Snapshot before any awaits.  _watcher_task's finally block sets
        # self._launch_proc = None at the first await point, so we must
        # hold a local reference and clear the attribute immediately.
        proc = self._launch_proc
        self._launch_proc = None

        for task in (self._reader_task, self._watcher_task):
            if task and not task.done():
                task.cancel()
        self._reader_task = None
        self._watcher_task = None

        if proc is not None:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass

            try:
                await asyncio.wait_for(proc.wait(), timeout=TERMINATE_TIMEOUT_S)
            except asyncio.TimeoutError:
                try:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

        await self._kill_stray_gazebo()

    async def _read_output(self, proc: asyncio.subprocess.Process) -> None:
        """Stream subprocess output to the log callback."""

        assert proc.stdout is not None
        try:
            async for raw_line in proc.stdout:
                line = raw_line.decode(errors="replace").rstrip()
                if not line:
                    continue
                if "[sim]" in line:
                    self._log(f"[cyan]{line}[/cyan]")
                elif "error" in line.lower() or "failed" in line.lower():
                    self._log(f"[red]{line}[/red]")
                elif "warn" in line.lower():
                    self._log(f"[yellow]{line}[/yellow]")
                else:
                    self._log(line)
        except asyncio.CancelledError:
            pass
        finally:
            if self._reader_task is asyncio.current_task():
                self._reader_task = None

    async def _watch_process(self, proc: asyncio.subprocess.Process) -> None:
        """Wait for ros2 launch to exit and update UI state once."""

        try:
            ret = await proc.wait()
        except asyncio.CancelledError:
            return
        finally:
            if self._launch_proc is proc:
                self._launch_proc = None
            if self._watcher_task is asyncio.current_task():
                self._watcher_task = None

        if self.sim_running:
            self._cleanup_generated_track(self._current_meta)
            self._set_stopped_state()
            if ret == 0:
                self._log("[yellow]Simulation ended cleanly (code 0).[/yellow]")
            else:
                self._log(
                    f"[red]Simulation exited with code {ret}. "
                    f"Check the log above for errors.[/red]"
                )

    async def _do_reset(self, cmd: list[str]) -> None:
        """Zero car velocity then teleport it to the saved start pose."""

        try:
            zero = await asyncio.create_subprocess_exec(
                "ros2",
                "topic",
                "pub",
                "--once",
                "/cmd_vel",
                "geometry_msgs/msg/Twist",
                "{}",
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
            await asyncio.wait_for(zero.wait(), timeout=RESET_CMD_VEL_TIMEOUT_S)
        except Exception:
            pass

        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            out, _ = await asyncio.wait_for(proc.communicate(), timeout=RESET_SERVICE_TIMEOUT_S)
            if proc.returncode == 0:
                self._log("[green]Car reset to start position.[/green]")
            else:
                self._log(f"[red]gz service error (code {proc.returncode}): {out.decode().strip()}[/red]")
        except asyncio.TimeoutError:
            self._log("[red]gz service timed out — is Gazebo running?[/red]")
        except FileNotFoundError:
            self._log("[red]gz not found — is Gazebo installed and sourced?[/red]")

    def _reset_runtime_state(self) -> None:
        self._start_pose = None
        self._current_meta = None

    def _set_stopped_state(self) -> None:
        self.sim_running = False
        self._reset_runtime_state()
        self._on_stopped()

    def _cleanup_generated_track(self, meta: dict | None) -> None:
        """Delete temporary generated map files when the metadata requests it."""

        if not meta or not meta.get("delete_on_stop"):
            return

        try:
            from fs_track_generator.artifacts import delete_generated_artifacts

            paths = _artifact_paths_from_meta(meta)
            if not paths:
                return
            delete_generated_artifacts(paths)
            version = meta.get("version", "?")
            self._log(f"[dim]Deleted temporary generated track v{version}.[/dim]")
        except Exception:
            self._log("[yellow]Could not delete generated track artifacts.[/yellow]")

    def _load_start_pose(self) -> None:
        if not self.sim_running:
            return

        meta = _read_latest_meta()
        if meta and "start_pose" in meta:
            self._current_meta = meta
            self._start_pose = meta["start_pose"]
            cone_count = meta.get("validation", {}).get("metrics", {}).get("cone_count", "?")
            track_info = (
                f"{meta.get('track_type', '?')}  "
                f"v{meta.get('version', '?')}  "
                f"seed={meta.get('seed', '?')}  "
                f"{meta.get('track_length_m', 0):.0f} m"
            )
            world_name = meta.get("world_name", DEFAULT_WORLD_NAME)
            render_mode = meta.get("render_mode", "high")
            self._on_track_ready(track_info, world_name)
            self._log(
                f"[green]Track ready[/green]  {track_info}  cones={cone_count}  render={render_mode}"
            )
        else:
            self._log("[yellow]Metadata not ready yet — retrying in 3 s…[/yellow]")
            self._schedule_timer(METADATA_RETRY_DELAY_S, self._load_start_pose)
