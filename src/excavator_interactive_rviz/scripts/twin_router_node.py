#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

from excavator_models import default_model, load_profile


class TwinRouterNode(Node):
    """
    AUWO Twin Router Node

    UI-level topics:
      /excavator_twin/commands  (Float64MultiArray)  <-- from RViz panel
      /excavator_twin/mode_cmd  (String)            <-- from RViz panel

    Unified state:
      /excavator_twin/state     (JointState)        --> to RViz, etc.

    Backends:
      Digital Twin Node (Gazebo):
        command: /arm_position_controller/commands
        state:   /joint_states

      Physical Twin Node (Novatron, future):
        command: /physical_twin/commands
        state:   /physical_twin/state

    Model info (latched, for the RViz Preset Poses panel):
      /excavator_twin/model     (String)  e.g. "v2"
      /excavator_twin/presets   (String)  one line per preset: "<name> body boom stick bucket",
                                          plus "cycle <name> <name> ..."

    Modes (string):
      "disabled"   -> block commands, do not republish state
      "simulation" -> commands -> sim only, state from sim
      "physical"   -> commands -> physical only, state from physical
      "dual"       -> commands -> both, state from sim (for now)
    """

    def __init__(self):
        super().__init__("twin_router_node")

        # Parameters (with reasonable defaults)
        self.declare_parameter("excavator_model", default_model())
        self.declare_parameter("default_mode", "simulation")
        self.declare_parameter("sim_command_topic", "/arm_position_controller/commands")
        self.declare_parameter("sim_state_topic", "/joint_states")
        self.declare_parameter("physical_command_topic", "/physical_twin/commands")
        self.declare_parameter("physical_state_topic", "/physical_twin/state")

        self.mode = self.get_parameter("default_mode").get_parameter_value().string_value
        if self.mode not in ("disabled", "simulation", "physical", "dual"):
            self.mode = "simulation"

        sim_cmd_topic = self.get_parameter("sim_command_topic").get_parameter_value().string_value
        sim_state_topic = self.get_parameter("sim_state_topic").get_parameter_value().string_value
        physical_cmd_topic = self.get_parameter("physical_command_topic").get_parameter_value().string_value
        physical_state_topic = self.get_parameter("physical_state_topic").get_parameter_value().string_value

        self.get_logger().info(
            f"Starting TwinRouterNode in mode='{self.mode}'\n"
            f"  SIM  command: {sim_cmd_topic}\n"
            f"  SIM  state:   {sim_state_topic}\n"
            f"  PHYS command: {physical_cmd_topic}\n"
            f"  PHYS state:   {physical_state_topic}"
        )

        # Publishers
        self.sim_command_pub = self.create_publisher(Float64MultiArray, sim_cmd_topic, 10)
        self.phys_command_pub = self.create_publisher(Float64MultiArray, physical_cmd_topic, 10)

        self.twin_state_pub = self.create_publisher(JointState, "/excavator_twin/state", 10)
        self.mode_pub = self.create_publisher(String, "/excavator_twin/mode", 10)

        # Subscriptions from UI
        self.twin_command_sub = self.create_subscription(
            Float64MultiArray,
            "/excavator_twin/commands",
            self.on_twin_command,
            10,
        )

        self.mode_cmd_sub = self.create_subscription(
            String,
            "/excavator_twin/mode_cmd",
            self.on_mode_cmd,
            10,
        )

        # Subscriptions from backends
        self.sim_state_sub = self.create_subscription(
            JointState,
            sim_state_topic,
            self.on_sim_state,
            10,
        )

        self.phys_state_sub = self.create_subscription(
            JointState,
            physical_state_topic,
            self.on_phys_state,
            10,
        )

        # Model-specific preset poses for the RViz panel (latched)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.model_pub = self.create_publisher(String, "/excavator_twin/model", latched)
        self.presets_pub = self.create_publisher(String, "/excavator_twin/presets", latched)
        self.publish_model_info(self.get_parameter("excavator_model").value)

        # Allow parameter-based mode change if desired
        self.add_on_set_parameters_callback(self.on_param_change)

        self.publish_mode()

    # ------------------------------------------------------------------ #
    # Model info                                                         #
    # ------------------------------------------------------------------ #

    def publish_model_info(self, model):
        try:
            profile = load_profile(model)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Cannot load excavator model '{model}': {e}")
            return
        lines = []
        for name, q in profile.get("presets", {}).items():
            lines.append(name + " " + " ".join(f"{float(v):.4f}" for v in q))
        seq = profile.get("cycle_sequence", [])
        if seq:
            lines.append("cycle " + " ".join(seq))
        self.model_pub.publish(String(data=profile["model"]))
        self.presets_pub.publish(String(data="\n".join(lines)))
        self.get_logger().info(
            f"Excavator model {profile['model']}: published {len(profile.get('presets', {}))} presets")

    # ------------------------------------------------------------------ #
    # Mode handling                                                      #
    # ------------------------------------------------------------------ #

    def on_param_change(self, params):
        """Allow changing default_mode as a runtime knob."""
        for p in params:
            if p.name == "default_mode":
                new_mode = p.value
                if new_mode not in ("disabled", "simulation", "physical", "dual"):
                    self.get_logger().warn(
                        f"Ignoring invalid default_mode '{new_mode}'"
                    )
                    return SetParametersResult(successful=False)
                self.mode = new_mode
                self.get_logger().info(f"Mode changed via param to '{self.mode}'")
                self.publish_mode()
        return SetParametersResult(successful=True)

    def on_mode_cmd(self, msg: String):
        """Handle UI mode commands (from RViz panel)."""
        new_mode = msg.data.strip().lower()
        if new_mode not in ("disabled", "simulation", "physical", "dual"):
            self.get_logger().warn(f"Ignoring invalid mode_cmd '{new_mode}'")
            return
        if new_mode != self.mode:
            self.get_logger().info(f"Mode changed via /excavator_twin/mode_cmd to '{new_mode}'")
            self.mode = new_mode
            self.publish_mode()

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Command routing                                                    #
    # ------------------------------------------------------------------ #

    def on_twin_command(self, msg: Float64MultiArray):
        if self.mode == "disabled":
            return

        if self.mode in ("simulation", "dual"):
            self.sim_command_pub.publish(msg)

        if self.mode in ("physical", "dual"):
            self.phys_command_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # State routing                                                      #
    # ------------------------------------------------------------------ #

    def on_sim_state(self, msg: JointState):
        # In simulation or dual mode, we use sim as the "visual" state source
        if self.mode in ("simulation", "dual"):
            self.twin_state_pub.publish(msg)

    def on_phys_state(self, msg: JointState):
        # In physical mode, real state should drive the twin
        if self.mode == "physical":
            self.twin_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwinRouterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

