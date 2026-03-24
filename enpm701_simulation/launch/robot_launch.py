import importlib.util
import os
import re

import launch
import launch.actions
import launch.event_handlers
import launch.events
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('enpm701_simulation')

    # --- Step 1: run generate_block_info to randomise block_info.yaml ---
    script_path = os.path.join(package_dir, 'scripts', 'generate_block_info.py')
    spec = importlib.util.spec_from_file_location('generate_block_info', script_path)
    gen_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen_mod)

    block_info_path = os.path.join(package_dir, 'worlds', 'block_info.yaml')
    gen_mod.generate(block_info_path)

    # --- Step 2: read generated block positions ---
    with open(block_info_path) as f:
        block_info = yaml.safe_load(f)

    # --- Step 3: build temp world with new blocks ---
    base_world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt')
    with open(base_world_path) as f:
        world_content = f.read()

    # Remove any existing Block entries (no nested braces in Block instances)
    world_content = re.sub(r'Block \{[^}]*\}', '', world_content, flags=re.DOTALL)
    world_content = world_content.rstrip() + '\n'

    # Append one Block node per entry in block_info.yaml
    for block_name in sorted(block_info.keys()):
        info = block_info[block_name]
        color = info['color']
        translation = info['translation']
        world_content += (
            f'Block {{\n'
            f'  name "{block_name}"\n'
            f'  translation {translation}\n'
            f'  color {color}\n'
            f'}}\n'
        )

    # Write alongside the protos so relative EXTERNPROTO paths still resolve
    temp_world_path = os.path.join(package_dir, 'worlds', 'my_world_generated.wbt')
    with open(temp_world_path, 'w') as f:
        f.write(world_content)

    print(f'[robot_launch] Generated world: {temp_world_path}')

    # --- Step 4: launch Webots with the generated world ---
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(world=temp_world_path)

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[{'robot_description': robot_description_path}],
    )

    return [
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        launch.actions.OpaqueFunction(function=launch_setup),
    ])
