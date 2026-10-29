## Holocean keyboard teleop launch file
# Author: Zouyixu

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from pathlib import Path

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation (keyboard teleop)')

    base = Path(get_package_share_directory('holoocean_examples'))
    params_file = base / 'config' / 'key_config.yaml'

    holoocean_namespace = 'holoocean'

    # The keyboard node is provided by https://github.com/cmower/ros2-keyboard
    # (start it with: ros2 run keyboard keyboard).  It is started here only if
    # the package was built in this workspace.
    try:
        get_package_share_directory('keyboard')
        keyboard_available = True
    except PackageNotFoundError:
        keyboard_available = False
        print('WARNING: package "keyboard" not found. Start a keyboard node that '
              'publishes keyboard/keydown and keyboard/keyup manually.')

    key_holo = launch_ros.actions.Node(
        name='key_holoocean',
        package='holoocean_examples',
        namespace=holoocean_namespace,
        executable='key_holoocean',
        output='screen',
        parameters=[params_file]
    )

    holoocean_main_node = launch_ros.actions.Node(
        name='holoocean_node',
        package='holoocean_main',
        executable='holoocean_node',
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    nodes = [holoocean_main_node, key_holo]

    if keyboard_available:
        nodes.append(launch_ros.actions.Node(
            package='keyboard',
            executable='keyboard',
            namespace=holoocean_namespace,
            name='keyboard',
            output='screen',
        ))

    return LaunchDescription(nodes)
