import argparse

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import asyncio
import omni.kit.asset_converter
from omni.kit.asset_converter import AssetConverterContext
from pxr import Usd,UsdGeom






def progress_callback(current_step: int, total: int):
    # Show progress
    print(f"{current_step} of {total}")

async def convert(input_asset_path, output_asset_path):
    task_manager = omni.kit.asset_converter.get_instance()
    converter_context = AssetConverterContext()
    converter_context.convert_stage_up_z = True
    converter_context.apply_up_axis_conversion = True  # << important line

    task = task_manager.create_converter_task(
    input_asset_path,
    output_asset_path,
    progress_callback,
    asset_converter_context=converter_context  # <-- Pass it here
    )

    success = await task.wait_until_finished()
    if not success:
        print("❌ Conversion failed")
        print("Status:", task.get_status())
        print("Error:", task.get_error_message())
    else:
        stage = Usd.Stage.Open(output_asset_path)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        print("Up Axis:", UsdGeom.GetStageUpAxis(stage))  # Should print 'Z'
        print("✅ Conversion succeeded")

        

# Main entry point
def main():
    parser = argparse.ArgumentParser(description="Convert assets in Isaac Sim using Asset Converter.")
    parser.add_argument("--input", required=True, help="Path to the input asset file")
    parser.add_argument("--output", required=True, help="Path to the output USD file")
    args = parser.parse_args()

    # Schedule the task
    asyncio.ensure_future(convert(args.input, args.output))

    # Keep Isaac Sim running until all tasks complete
    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()

if __name__ == "__main__":
    main()