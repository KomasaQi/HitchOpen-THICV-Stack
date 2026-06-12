#!/usr/bin/env python3
"""
Load a 3 km x 3 km, 6-degree sloped OpenDRIVE map with collidable guardrails into CARLA.

Usage:
  1) Start the CARLA server first.
     Linux:   ./CarlaUE4.sh
     Windows: CarlaUE4.exe

  2) Run this script from any folder where the CARLA Python API can be imported:
     python run_square_slope_3km_guardrails.py --xodr square_slope_3km_6deg_guardrails.xodr

Optional:
     python run_square_slope_3km_guardrails.py --xodr square_slope_3km_6deg_guardrails.xodr --spawn-vehicle

Notes:
  - The reliable continuous collision boundary is CARLA's OpenDRIVE-generated boundary wall,
    controlled by --wall-height. The default is 1.2 m.
  - The script also tries to spawn visible static barrier props around all four sides by default.
    Those props are mainly for visualization; collision safety is provided by --wall-height.
  - For a 3 km x 3 km map, dense visible props can be expensive. Increase --guardrail-spacing
    if your frame rate drops, or use --no-prop-guardrails to keep only the collision wall.
"""

import argparse
import math
import os
import random
import sys
from typing import List, Optional

try:
    import carla
except ImportError as exc:
    raise SystemExit(
        "Cannot import the CARLA Python API. Run this script from CARLA/PythonAPI, "
        "or add CARLA's PythonAPI/carla egg to PYTHONPATH."
    ) from exc

MAP_LENGTH = 3000.0
MAP_HALF_WIDTH = 1500.0
SLOPE_DEG = 6.0
SLOPE = math.tan(math.radians(SLOPE_DEG))


def road_z(x: float) -> float:
    """Road elevation at longitudinal coordinate x."""
    return SLOPE * x


def build_generation_parameters(args):
    """Create OpenDRIVE mesh generation parameters for CARLA."""
    params = carla.OpendriveGenerationParameters()
    params.vertex_distance = args.vertex_distance
    params.max_road_length = args.max_road_length
    params.wall_height = args.wall_height
    params.additional_width = args.additional_width
    params.smooth_junctions = True
    params.enable_mesh_visibility = True

    # Present in some CARLA versions; harmlessly skipped in older versions.
    if hasattr(params, "enable_pedestrian_navigation"):
        params.enable_pedestrian_navigation = True

    return params


def set_spectator(world):
    """Put the spectator camera above the 3 km x 3 km sloped square."""
    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            carla.Location(x=MAP_LENGTH * 0.50, y=0.0, z=900.0),
            carla.Rotation(pitch=-60.0, yaw=0.0, roll=0.0),
        )
    )


def try_spawn_vehicle(world):
    """Optionally spawn one vehicle near the lower side of the slope."""
    blueprint_library = world.get_blueprint_library()
    vehicles = blueprint_library.filter("vehicle.*")
    if not vehicles:
        print("No vehicle blueprint found.")
        return None

    blueprint = random.choice(vehicles)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", "ego")

    x = 30.0
    y = -MAP_HALF_WIDTH * 0.25
    spawn_transform = carla.Transform(
        carla.Location(x=x, y=y, z=road_z(x) + 1.5),
        carla.Rotation(pitch=SLOPE_DEG, yaw=0.0, roll=0.0),
    )

    vehicle = world.try_spawn_actor(blueprint, spawn_transform)
    if vehicle is None:
        print("Vehicle spawn failed. The map and guardrail boundary were still loaded successfully.")
        return None

    vehicle.set_autopilot(False)
    print(f"Spawned vehicle: {vehicle.type_id}")
    return vehicle


def find_guardrail_blueprint(blueprint_library):
    """Find a static barrier-like blueprint in the current CARLA build."""
    preferred_filters = [
        "static.prop.streetbarrier",
        "static.prop.*barrier*",
        "static.prop.warningconstruction",
        "static.prop.*warning*",
        "static.prop.trafficcone",
        "static.prop.*cone*",
    ]

    candidates = []
    seen = set()
    for pattern in preferred_filters:
        for bp in blueprint_library.filter(pattern):
            if bp.id not in seen:
                candidates.append(bp)
                seen.add(bp.id)

    if not candidates:
        return None

    # Prefer real barrier blueprints over cones/warning props when available.
    candidates.sort(key=lambda bp: ("barrier" not in bp.id.lower(), "warning" not in bp.id.lower(), bp.id))
    return candidates[0]


def spawn_guardrail_props(world, spacing: float, inset: float, prop_z_offset: float) -> List[object]:
    """Spawn visible static props around all four edges of the square slope."""
    blueprint_library = world.get_blueprint_library()
    blueprint = find_guardrail_blueprint(blueprint_library)
    if blueprint is None:
        print("No static barrier-like blueprint found. Using OpenDRIVE wall collision only.")
        return []

    spawned = []
    failed = 0

    def try_spawn(x: float, y: float, yaw: float, pitch: float) -> None:
        nonlocal failed
        transform = carla.Transform(
            carla.Location(x=x, y=y, z=road_z(x) + prop_z_offset),
            carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0),
        )
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is None:
            failed += 1
        else:
            spawned.append(actor)

    # Long sides: props follow the 6-degree uphill direction.
    y_left = MAP_HALF_WIDTH - inset
    y_right = -MAP_HALF_WIDTH + inset
    x = spacing / 2.0
    while x < MAP_LENGTH:
        try_spawn(x, y_left, yaw=0.0, pitch=SLOPE_DEG)
        try_spawn(x, y_right, yaw=180.0, pitch=SLOPE_DEG)
        x += spacing

    # Cross sides: props run across the width. Elevation is constant across y for fixed x.
    x_lower = inset
    x_upper = MAP_LENGTH - inset
    y = -MAP_HALF_WIDTH + spacing / 2.0
    while y < MAP_HALF_WIDTH:
        try_spawn(x_lower, y, yaw=90.0, pitch=0.0)
        try_spawn(x_upper, y, yaw=90.0, pitch=0.0)
        y += spacing

    print(f"Guardrail prop blueprint: {blueprint.id}")
    print(f"Spawned visible guardrail props: {len(spawned)}")
    if failed:
        print(f"Visible guardrail prop spawn failures, usually due to overlap/collision: {failed}")
    return spawned


def main():
    parser = argparse.ArgumentParser(
        description="Load a 3 km x 3 km, 6-degree slope .xodr map with collidable guardrails into CARLA."
    )
    parser.add_argument("--host", default="127.0.0.1", help="CARLA server host. Default: 127.0.0.1")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port. Default: 2000")
    parser.add_argument("--timeout", type=float, default=60.0, help="CARLA client timeout in seconds. Default: 60")
    parser.add_argument("--xodr", default="square_slope_3km_6deg_guardrails.xodr", help="Path to the .xodr file.")
    parser.add_argument("--spawn-vehicle", action="store_true", help="Spawn one test vehicle on the slope.")

    # Mesh generation settings. wall_height creates the reliable continuous collision boundary.
    parser.add_argument("--vertex-distance", type=float, default=10.0, help="Mesh vertex distance in meters. Default: 10")
    parser.add_argument("--max-road-length", type=float, default=100.0, help="Maximum mesh road segment length in meters. Default: 100")
    parser.add_argument("--wall-height", type=float, default=1.2, help="Collidable boundary wall height in meters. Default: 1.2")
    parser.add_argument("--additional-width", type=float, default=0.0, help="Extra mesh width on each side in meters. Default: 0")

    # Visible guardrail props. These are optional because CARLA builds differ in available static prop blueprints.
    parser.add_argument("--no-prop-guardrails", action="store_true", help="Do not spawn visible static guardrail props.")
    parser.add_argument("--guardrail-spacing", type=float, default=25.0, help="Spacing between visible guardrail props in meters. Default: 25")
    parser.add_argument("--guardrail-inset", type=float, default=1.0, help="How far inside the square edge to place props. Default: 1")
    parser.add_argument("--guardrail-z-offset", type=float, default=0.55, help="Vertical offset for visible props above road surface. Default: 0.55")
    args = parser.parse_args()

    if not os.path.exists(args.xodr):
        raise SystemExit(f"OpenDRIVE file not found: {args.xodr}")

    if args.guardrail_spacing <= 0.0:
        raise SystemExit("--guardrail-spacing must be greater than 0")

    if args.wall_height <= 0.0:
        print("Warning: --wall-height <= 0 disables or weakens the continuous collision guardrail boundary.")

    with open(args.xodr, "r", encoding="utf-8") as file:
        xodr_data = file.read()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    params = build_generation_parameters(args)
    world = client.generate_opendrive_world(xodr_data, params)
    set_spectator(world)

    guardrail_actors = []
    if not args.no_prop_guardrails:
        guardrail_actors = spawn_guardrail_props(
            world,
            spacing=args.guardrail_spacing,
            inset=args.guardrail_inset,
            prop_z_offset=args.guardrail_z_offset,
        )

    if args.spawn_vehicle:
        try_spawn_vehicle(world)

    print("Loaded OpenDRIVE map successfully.")
    print("Map: 3000 m long x 3000 m wide square slope")
    print("Slope angle: 6 degrees")
    print("Height gain over 3000 m: %.3f m" % road_z(MAP_LENGTH))
    print("Continuous boundary collision wall height: %.2f m" % args.wall_height)
    if guardrail_actors:
        print("Visible guardrail props were spawned around all four sides.")
    else:
        print("Visible guardrail props were not spawned; OpenDRIVE wall collision is still active if --wall-height > 0.")
    print("Keep the CARLA server running to inspect or control the map.")


if __name__ == "__main__":
    main()
