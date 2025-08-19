
import argparse
from isaacsim import SimulationApp

# Launch Isaac Sim headless
simulation_app = SimulationApp({"headless": True})

# Now import Omniverse/Isaac Lab stuff
from isaaclab.sim.converters import MeshConverter, MeshConverterCfg


def main():
    parser = argparse.ArgumentParser(description="Convert mesh files (OBJ/STL/FBX) to USD using Isaac Lab MeshConverter.")
    parser.add_argument("--input", required=True, help="Path to input mesh file (e.g., .obj, .stl, .fbx)")
    parser.add_argument("--output_dir", required=True, help="Directory to store output USD file")
    parser.add_argument("--output_name", default=None, help="Name of the output USD file (e.g., my_asset.usd)")
    parser.add_argument("--force", action="store_true", help="Force conversion even if USD already exists")
    parser.add_argument("--scale", type=float, nargs=3, default=(1.0, 1.0, 1.0), help="Scale to apply (x y z)")
    parser.add_argument("--translation", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="Translation to apply (x y z)")
    parser.add_argument(
        "--rotation", type=float, nargs=4,
        default=(0.707, 0.707, 0.0, 0.0),  # 90° around X: Y-up → Z-up
        help="Quaternion rotation to apply (w x y z)"
    )
    parser.add_argument("--instanceable", action="store_true", help="Make output USD instanceable")

    args = parser.parse_args()

    cfg = MeshConverterCfg(
        asset_path=args.input,
        usd_dir=args.output_dir,
        usd_file_name=args.output_name,
        force_usd_conversion=args.force,
        scale=tuple(args.scale),
        translation=tuple(args.translation),
        rotation=tuple(args.rotation),
        make_instanceable=args.instanceable,
    )

    # Convert the asset
    converter = MeshConverter(cfg)

    print("✅ USD asset created at:", converter.usd_path)

    simulation_app.close()


if __name__ == "__main__":
    main()
