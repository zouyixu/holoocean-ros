#!/usr/bin/env python3
"""Keyboard → AgentCommand converter for torpedo-style AUVs.

Key events are read from a ROS keyboard node that publishes raw events on
``keyboard/keydown`` and ``keyboard/keyup`` (the ``keyboard`` package,
https://github.com/cmower/ros2-keyboard).  If ``keyboard_msgs`` is not
installed the node falls back to ``std_msgs/String`` (payload = key name),
which keeps it usable for testing with ``ros2 topic pub``.

The published command ('command/agent', frame_id = agent name) follows the
torpedo fossen model, five values, one per control input:

    index 0 : pitch fin angle       index 2 : mirror of index 0 (-index 0)
    index 1 : yaw fin angle         index 3 : mirror of index 1 (-index 1)
    index 4 : thruster command

Fins are limited to +-fin.max_angle, the thruster to +-thrust.max:

    bow up     → [0] = -fin_max, [2] = +fin_max
    bow down   → [0] = +fin_max, [2] = -fin_max
    turn left  → [1] = -fin_max, [3] = +fin_max
    turn right → [1] = +fin_max, [3] = -fin_max
    forward    → [4] = +thrust_max, backward → [4] = -thrust_max

Key map (press = active, release = back to neutral):

    Up / Down     forward / backward thrust      (index 4)
    Left / Right  turn left / turn right         (index 1, 3)
    i / k         bow up / bow down              (index 0, 2)
    Space         arm / disarm the active agent
    r             reset the simulation
    1 .. 9        select which agent is driven

NOTE: ``torpedo_fossen.json`` starts in ``depthHeadingAutopilot``; direct u
commands only take effect in ``manualControl``:

    ros2 service call /holoocean/control_mode holoocean_interfaces/srv/SetControlMode \\
        "{vehicle_name: 'auv0', mode: 0}"
"""

import json
import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from holoocean_interfaces.msg import AgentCommand
from ament_index_python.packages import get_package_share_directory

try:
    from keyboard_msgs.msg import Key as _KeyMsg
    HAS_KEYBOARD_MSGS = True
except ImportError:
    from std_msgs.msg import String as _KeyMsg
    HAS_KEYBOARD_MSGS = False


TORPEDO_AGENT_TYPES = ('TorpedoAUV', 'CougUV')

#  action name → key aliases (compared after _normalize())
KEY_ALIASES = {
    'forward':  ('up', 'arrowup'),
    'backward': ('down', 'arrowdown'),
    'left':     ('left', 'arrowleft'),
    'right':    ('right', 'arrowright'),
    'bow_up':   ('i',),
    'bow_down': ('k',),
}


def _normalize(name: str) -> str:
    """Normalize a key name so 'Up', 'up_arrow' and 'ArrowUp' compare equal."""
    return ''.join(ch for ch in name.lower() if ch.isalnum())


def _key_name(msg) -> str:
    """Extract the normalized key name from keyboard_msgs/Key or std_msgs/String."""
    return _normalize(getattr(msg, 'key', getattr(msg, 'data', '')))


#  Per-agent controller

class AgentController:
    """Holds the pressed keys of one agent and builds its torpedo command."""

    def __init__(self, name: str, agent_type: str,
                 fin_max: float, thrust_max: float):
        self.name = name
        self.type = agent_type
        self.fin_max = fin_max
        self.thrust_max = thrust_max

        # Mutable state: actions currently held down
        self.pressed: set[str] = set()

    def reset(self):
        """Release every key (call on agent switch or shutdown)."""
        self.pressed.clear()

    def press(self, action: str):
        self.pressed.add(action)

    def release(self, action: str):
        self.pressed.discard(action)

    def agent_command(self) -> AgentCommand:
        """Build the five element torpedo command from the held keys."""
        pitch = self._sign('bow_up', 'bow_down')      # bow up → index 0 negative
        yaw   = self._sign('left', 'right')           # turn left → index 1 negative
        thrust = self._sign('backward', 'forward')

        fin_pitch = pitch * self.fin_max
        fin_yaw = yaw * self.fin_max

        cmd = AgentCommand()
        cmd.header.frame_id = self.name
        cmd.command = [
            fin_pitch,
            fin_yaw,
            -fin_pitch,
            -fin_yaw,
            thrust * self.thrust_max,
        ]
        return cmd

    def _sign(self, negative: str, positive: str) -> int:
        """-1 / 0 / +1 depending on which of the two opposite keys is held."""
        return int(positive in self.pressed) - int(negative in self.pressed)


#  ROS 2 node

class KeyToAgentCommand(Node):
    def __init__(self):
        super().__init__('key_holoocean')

        # Publisher
        self.agent_pub = self.create_publisher(AgentCommand, 'command/agent', 10)

        self.reset_client = self.create_client(Trigger, 'reset')

        # Parameters
        self.declare_parameter('fin.max_angle', 45.0)
        self.declare_parameter('thrust.max', 100.0)
        self.declare_parameter('publish_rate', 10.0)

        self.declare_parameter('key.down_topic', 'keyboard/keydown')
        self.declare_parameter('key.up_topic', 'keyboard/keyup')
        self.declare_parameter('key.reset', 'r')
        self.declare_parameter('key.toggle_enable', 'space')

        self.declare_parameter('relative_path',  True)
        self.declare_parameter('scenario_path',  '')

        self.declare_parameter('agents.default', 'auv0')

        #  Read parameters
        def gf(p): return self.get_parameter(p).get_parameter_value().double_value
        def gs(p): return self.get_parameter(p).get_parameter_value().string_value

        fin_max = abs(gf('fin.max_angle'))
        thrust_max = abs(gf('thrust.max'))

        self.key_map: dict[str, str] = {}
        for action, aliases in KEY_ALIASES.items():
            for alias in aliases:
                self.key_map[_normalize(alias)] = action
        self.reset_key = _normalize(gs('key.reset'))
        self.toggle_key = _normalize(gs('key.toggle_enable'))

        # Load agent names and types from the scenario JSON.
        scenario_path = gs('scenario_path')
        relative_path = self.get_parameter('relative_path').get_parameter_value().bool_value
        if relative_path:
            scenario_path = os.path.join(
                get_package_share_directory('holoocean_main'), scenario_path)
        # Check path exists
        if not os.path.isfile(scenario_path):
            raise ValueError(f"Scenario file not found with path: {scenario_path}")
        with open(scenario_path, 'r') as f:
            scenario = json.load(f)

        scenario_agents = scenario.get('agents', [])
        if not scenario_agents:
            raise ValueError(f'No agents found in scenario file: {scenario_path}')

        # Build controllers, one per agent in the scenario
        self.controllers: dict[str, AgentController] = {}
        self.select_map: dict[str, AgentController] = {}

        for i, agent in enumerate(scenario_agents):
            name = agent['agent_name']
            atype = agent['agent_type']
            ctrl = AgentController(name, atype, fin_max, thrust_max)
            self.controllers[name] = ctrl
            self.select_map[str(i + 1)] = ctrl
            self.get_logger().info(
                f'  agent [{name}]  type={atype}  select key={i + 1}')
            if atype not in TORPEDO_AGENT_TYPES:
                self.get_logger().warn(
                    f'agent [{name}] has type {atype}; the five element torpedo '
                    'command may not match its control scheme.')

        default = gs('agents.default')
        if default not in self.controllers:
            default = scenario_agents[0]['agent_name']
        self.active: AgentController = self.controllers[default]

        self.enabled = True  # Start enabled for convenience
        self.warned_empty_key = False

        #  Keyboard events
        topic_down = gs('key.down_topic')
        topic_up = gs('key.up_topic')
        if HAS_KEYBOARD_MSGS:
            self.get_logger().info('Using keyboard_msgs/Key')
        else:
            self.get_logger().warn(
                'keyboard_msgs not found, falling back to std_msgs/String. '
                'Build https://github.com/cmower/ros2-keyboard for the standard '
                'keyboard node, or publish String messages to the topics below.')

        self.create_subscription(_KeyMsg, topic_down, self._key_down_cb, 10)
        self.create_subscription(_KeyMsg, topic_up, self._key_up_cb, 10)

        # Re-send the current command so a held key keeps driving the vehicle
        rate = gf('publish_rate')
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Key node ready on {topic_down}/{topic_up}.  '
            f'Default agent: {self.active.name}  '
            f'fin=+-{fin_max:g}  thrust=+-{thrust_max:g}'
        )

    #  Key callbacks

    def _key_down_cb(self, msg):
        name = _key_name(msg)
        if not name and not self.warned_empty_key:
            self.warned_empty_key = True
            self.get_logger().warn(
                'Key event without a readable key name. Check the field name of '
                'the keyboard message type in use.')
        self._handle_key(name, pressed=True)

    def _key_up_cb(self, msg):
        self._handle_key(_key_name(msg), pressed=False)

    def _handle_key(self, key: str, pressed: bool):
        if not key:
            return

        # Agent selection
        ctrl = self.select_map.get(key)
        if ctrl is not None:
            if pressed and ctrl is not self.active:
                previous = self.active
                previous.reset()
                self.agent_pub.publish(previous.agent_command())  # stop the old agent
                self.active = ctrl
                self.get_logger().info(f'Active agent → {ctrl.name} ({ctrl.type})')
            return

        # Arm / disarm (stops the vehicle when disarmed)
        if key == self.toggle_key:
            if pressed:
                self.enabled = not self.enabled
                self.active.reset()
                self.get_logger().info('ENABLED' if self.enabled else 'DISABLED')
                self.agent_pub.publish(self.active.agent_command())
            return

        # Reset the simulation
        if key == self.reset_key:
            if pressed:
                self.reset_client.call_async(Trigger.Request())
                for controller in self.controllers.values():
                    controller.reset()
                self.get_logger().info('RESET')
            return

        action = self.key_map.get(key)
        if action is None:
            return

        if pressed:
            self.active.press(action)
        else:
            self.active.release(action)
        self._publish()

    #  Publishing

    def _publish(self):
        if not self.enabled:
            return
        self.agent_pub.publish(self.active.agent_command())

    def stop(self):
        """Send neutral commands so the vehicles do not keep their last input."""
        for controller in self.controllers.values():
            controller.reset()
            self.agent_pub.publish(controller.agent_command())


def main(args=None):
    rclpy.init(args=args)
    node = KeyToAgentCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
