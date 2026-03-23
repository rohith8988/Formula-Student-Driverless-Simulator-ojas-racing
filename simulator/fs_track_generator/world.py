"""Turn a generated track layout into a Gazebo world file."""

from __future__ import annotations

from .artifacts import resolve_track_mesh_dir
from .models import Cone, TrackLayout


# ── Cone rule dimensions (Formula Student Germany event handbook) ─────────────
# Small boundary cones: 228 × 228 × 325 mm.
# Large orange cones:   285 × 285 × 505 mm.
SMALL_CONE_RULE_DIAMETER_M = 0.228
SMALL_CONE_RULE_HEIGHT_M   = 0.325
SMALL_CONE_MESH_DIAMETER_M = 0.2253952
SMALL_CONE_MESH_HEIGHT_M   = 0.3034077

LARGE_CONE_RULE_DIAMETER_M = 0.285
LARGE_CONE_RULE_HEIGHT_M   = 0.505
LARGE_CONE_MESH_DIAMETER_M = 0.2613056457884
LARGE_CONE_MESH_HEIGHT_M   = 0.5254544448352753

# ── Cone physics ──────────────────────────────────────────────────────────────
# Cones are dynamic (not static) so they react to the car.  The values below
# match real FSG hardware as closely as practical.
#
# Mass: actual plastic traffic cones are ~0.8 kg (small) / ~1.5 kg (large).
#
# Inertia: approximated as a solid cylinder using the standard formula:
#   Ixx = Iyy = m*(3r² + h²)/12  (tipping axes)
#   Izz       = m*r²/2            (spin axis)
# The exact shape is a frustum, but a cylinder of the same radius and height
# gives inertia values that feel correct in simulation.
#
# Damping: keeps knocked cones from sliding/spinning indefinitely across the
# track.  These are per-link velocity decay coefficients in Gazebo.
#
# Friction (mu): cone-on-asphalt is slightly lower than tyre-on-asphalt.
# 0.6 is a common value used in EUFS and similar FS simulators.
SMALL_CONE_MASS_KG = 0.8
LARGE_CONE_MASS_KG = 1.5

_CONE_LINEAR_DAMPING  = 0.3   # translational drag  (m/s decay)
_CONE_ANGULAR_DAMPING = 0.5   # rotational drag     (rad/s decay)
_CONE_FRICTION        = 0.6   # cone surface mu (isotropic)


def _cone_inertia(mass: float, radius: float, height: float) -> tuple[float, float, float]:
    """Return (Ixx, Iyy, Izz) for a solid cylinder with the given dimensions."""
    ixx = mass * (3.0 * radius**2 + height**2) / 12.0
    izz = mass * radius**2 / 2.0
    return ixx, ixx, izz


def _cone_mesh_name(cone: Cone) -> str:
    if cone.size == "large":
        return "cone_big.dae"
    if cone.color == "blue":
        return "cone_blue.dae"
    if cone.color == "yellow":
        return "cone_yellow.dae"
    return "cone.dae"


def _cone_dimensions(cone: Cone) -> tuple[tuple[float, float, float], float, float]:
    if cone.size == "large":
        scale_xy = LARGE_CONE_RULE_DIAMETER_M / LARGE_CONE_MESH_DIAMETER_M
        scale_z = LARGE_CONE_RULE_HEIGHT_M / LARGE_CONE_MESH_HEIGHT_M
        return ((scale_xy, scale_xy, scale_z), LARGE_CONE_RULE_DIAMETER_M / 2.0, LARGE_CONE_RULE_HEIGHT_M)

    scale_xy = SMALL_CONE_RULE_DIAMETER_M / SMALL_CONE_MESH_DIAMETER_M
    scale_z = SMALL_CONE_RULE_HEIGHT_M / SMALL_CONE_MESH_HEIGHT_M
    return ((scale_xy, scale_xy, scale_z), SMALL_CONE_RULE_DIAMETER_M / 2.0, SMALL_CONE_RULE_HEIGHT_M)


def _cone_material(cone: Cone) -> tuple[str, str, str]:
    if cone.size == "large" or cone.color == "orange":
        color = "0.86 0.45 0.12 1"
    elif cone.color == "blue":
        color = "0.12 0.35 0.86 1"
    else:
        color = "0.86 0.79 0.12 1"

    return color, color, "0.05 0.05 0.05 1"


def _cone_visual(cone: Cone, mesh_dir, render_mode: str) -> str:
    mesh_scale, collision_radius, collision_height = _cone_dimensions(cone)

    if render_mode == "performance":
        ambient, diffuse, specular = _cone_material(cone)
        return f"""
        <visual name="visual">
          <pose>0 0 {collision_height / 2.0:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{collision_radius:.4f}</radius>
              <length>{collision_height:.4f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>{specular}</specular>
          </material>
          <cast_shadows>false</cast_shadows>
        </visual>"""

    mesh_uri = (mesh_dir / _cone_mesh_name(cone)).as_uri()
    return f"""
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
              <scale>{mesh_scale[0]:.6f} {mesh_scale[1]:.6f} {mesh_scale[2]:.6f}</scale>
            </mesh>
          </geometry>
        </visual>"""


def _cone_model(cone: Cone, index: int, mesh_dir, render_mode: str) -> str:
    _, collision_radius, collision_height = _cone_dimensions(cone)
    mass = LARGE_CONE_MASS_KG if cone.size == "large" else SMALL_CONE_MASS_KG
    ixx, iyy, izz = _cone_inertia(mass, collision_radius, collision_height)
    # CoM sits at the geometric centre of the cylinder.
    # The link origin is at ground level (z=0), so com_z = half the height.
    com_z = collision_height / 2.0

    return f"""
    <model name="cone_{index}">
      <!-- allow_auto_disable: physics engine puts this cone to sleep once it
           settles after being knocked.  A sleeping body costs almost nothing —
           same as static — and wakes up automatically on next contact. -->
      <allow_auto_disable>true</allow_auto_disable>
      <pose>{cone.x:.4f} {cone.y:.4f} 0 0 0 0</pose>
      <link name="link">
        <inertial>
          <pose>0 0 {com_z:.4f} 0 0 0</pose>
          <mass>{mass}</mass>
          <inertia>
            <ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{iyy:.6f}</iyy><iyz>0</iyz>
            <izz>{izz:.6f}</izz>
          </inertia>
        </inertial>
        <velocity_decay>
          <linear>{_CONE_LINEAR_DAMPING}</linear>
          <angular>{_CONE_ANGULAR_DAMPING}</angular>
        </velocity_decay>
        <collision name="collision">
          <pose>0 0 {com_z:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{collision_radius:.4f}</radius>
              <length>{collision_height:.4f}</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>{_CONE_FRICTION}</mu>
                <mu2>{_CONE_FRICTION}</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
{_cone_visual(cone, mesh_dir, render_mode)}
      </link>
    </model>"""


def build_world_sdf(
    layout: TrackLayout,
    *,
    world_name: str = "fs_world",
    render_mode: str = "high",
) -> str:
    """Create a complete Gazebo world file for a generated track."""

    # Ground friction coefficients sourced from FSAE Tire Test Consortium data
    # for Hoosier-type racing slicks on tarmac.
    # Wheels are set to μ = 2.0, so the ground value is the limiting factor:
    #   effective grip = min(ground_mu, wheel_mu) = min(ground_mu, 2.0) = ground_mu.
    #   dry:  μ = 1.3  — warm slick on clean, dry asphalt (mid-range value)
    #   wet:  μ = 0.5  — conservative estimate; slicks are very vulnerable to water
    friction = "0.5" if layout.weather == "wet" else "1.3"
    mesh_dir = resolve_track_mesh_dir()
    start = layout.start_pose
    camera_distance = 16.0
    camera_x = start.x - camera_distance
    camera_y = start.y
    camera_z = 8.0
    cone_models = "\n".join(
        _cone_model(cone, index, mesh_dir, render_mode)
        for index, cone in enumerate(layout.cones, start=1)
    )
    shadows = "false" if render_mode == "performance" else "true"

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{world_name}">
    <!-- 10 ms step = 100 Hz physics.  FS cars run at < 20 m/s so 100 Hz gives
         plenty of accuracy while reducing solver load by 20% vs the old 125 Hz. -->
    <physics name="10ms" type="ignored">
      <max_step_size>0.010</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <!-- Contact plugin removed: nothing in the pipeline subscribes to contact
         topics, so it was tracking and publishing ~350 cone-ground contact pairs
         per physics step for no benefit. Re-add when cone-hit detection is needed. -->

    <gui fullscreen="false">
      <plugin name="3D View" filename="GzScene3D">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">0</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.6 0.6 0.6 1</ambient_light>
        <background_color>0.9 0.9 0.9 1</background_color>
        <camera_pose>{camera_x:.3f} {camera_y:.3f} {camera_z:.3f} 0 0.42 0</camera_pose>
      </plugin>
      <plugin name="World control" filename="WorldControl">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">0</property>
          <property type="bool" key="resizable">0</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>1</play_pause>
        <step>1</step>
        <start_paused>0</start_paused>
      </plugin>
      <plugin name="World stats" filename="WorldStats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">0</property>
          <property type="bool" key="resizable">0</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>1</sim_time>
        <real_time>1</real_time>
        <real_time_factor>1</real_time_factor>
        <iterations>1</iterations>
      </plugin>
      <plugin name="Entity tree" filename="EntityTree"/>
    </gui>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>true</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>{shadows}</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>{shadows}</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>600 600</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>{friction}</mu>
                <mu2>{friction}</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>600 600</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.25 0.25 0.25 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
{cone_models}
  </world>
</sdf>
"""
