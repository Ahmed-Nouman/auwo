#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from excavator_models import default_model, load_profile


class ExcavatorJointInteractive(Node):
    """
    Interactive markers to control ALL excavator joints with linear sliders:

      body_rotation   (slider mapped to [-pi, pi])
      boom_rotation   (slider, URDF limits)
      stick_rotation  (slider, URDF limits)
      bucket_rotation (slider, URDF limits)

    Flow:
      RViz interactive markers --> this node --> /arm_position_controller/commands
      --> ros2_control + Gazebo --> /joint_states --> RViz robot model
    """

    def __init__(self):
        super().__init__("excavator_joint_interactive")

        # Excavator model (v1/v2): joint ranges come from its model profile
        model = self.declare_parameter("excavator_model", default_model()).value
        profile = load_profile(model)
        lim = profile["ui_limits"]

        # Joint order must match controllers.yaml
        self.joint_order = [
            "body_rotation",
            "boom_rotation",
            "stick_rotation",
            "bucket_rotation",
        ]

        # Current joint positions (updated from /joint_states)
        self.joint_positions = {name: 0.0 for name in self.joint_order}

        # === CONFIG: joint limits + marker frames + marker range ===
        # Limits from the model profile (ui_limits; v1 values are the previous hard-coded ones)
        self.joint_configs = {
            "body_rotation": {
                "frame_id": "body",          # link name in URDF
                "marker_name": "body_control",
                "description": "Body rotation",
                # practical range for the wide-range swing joint
                "joint_lower": float(lim["body_rotation"][0]),
                "joint_upper": float(lim["body_rotation"][1]),
                "marker_min":  -0.7,
                "marker_max":   0.7,
            },
            "boom_rotation": {
                "frame_id": "boom",
                "marker_name": "boom_control",
                "description": "Boom rotation",
                "joint_lower": float(lim["boom_rotation"][0]),
                "joint_upper": float(lim["boom_rotation"][1]),
                "marker_min":  -0.5,
                "marker_max":   0.5,
            },
            "stick_rotation": {
                "frame_id": "stick",
                "marker_name": "stick_control",
                "description": "Stick rotation",
                "joint_lower": float(lim["stick_rotation"][0]),
                "joint_upper": float(lim["stick_rotation"][1]),
                "marker_min":  -0.5,
                "marker_max":   0.5,
            },
            "bucket_rotation": {
                "frame_id": "bucket",
                "marker_name": "bucket_control",
                "description": "Bucket rotation",
                "joint_lower": float(lim["bucket_rotation"][0]),
                "joint_upper": float(lim["bucket_rotation"][1]),
                "marker_min":  -0.5,
                "marker_max":   0.5,
            },
        }

        # Map marker_name -> joint_name
        self.marker_to_joint = {
            cfg["marker_name"]: joint_name
            for joint_name, cfg in self.joint_configs.items()
        }

        # Subscribe to joint_states to stay in sync with Gazebo
        self.create_subscription(JointState, "joint_states", self.joint_state_cb, 10)

        # Publisher to the group position controller
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/arm_position_controller/commands",
            10,
        )

        # Interactive marker server
        self.server = InteractiveMarkerServer(self, "excavator_joint_server")

        # Create markers for all joints
        self.create_all_markers()

        self.get_logger().info(
            f"Excavator joint interactive markers (linear) started for model {profile['model']}.")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def joint_state_cb(self, msg: JointState) -> None:
        """Keep local copy of joint positions so we don't overwrite other joints."""
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = float(pos)

    def create_all_markers(self) -> None:
        """Create and insert linear interactive markers for all configured joints."""
        for joint_name, cfg in self.joint_configs.items():
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = cfg["frame_id"]
            int_marker.name = cfg["marker_name"]
            int_marker.description = cfg["description"]

            # Marker pose relative to the given frame
            int_marker.pose = Pose()
            int_marker.pose.position.x = 0.0
            int_marker.pose.position.y = 0.0
            int_marker.pose.position.z = 0.0

            # 1D linear slider along X axis
            control = InteractiveMarkerControl()
            control.name = cfg["marker_name"] + "_move"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

            # Axis orientation: X axis
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0

            int_marker.controls.append(control)

            # Insert with feedback callback (keyword arg for Jazzy)
            self.server.insert(int_marker, feedback_callback=self.process_feedback)

            self.get_logger().info(
                f"Created linear marker '{cfg['marker_name']}' for joint '{joint_name}' "
                f"on frame '{cfg['frame_id']}'"
            )

        self.server.applyChanges()

    def process_feedback(self, feedback) -> None:
        """
        Called whenever any marker is moved.

        Uses X translation of the marker, maps [marker_min, marker_max]
        to the joint's [joint_lower, joint_upper].
        """
        marker_name = feedback.marker_name

        if marker_name not in self.marker_to_joint:
            self.get_logger().warn(f"Unknown marker_name in feedback: {marker_name}")
            return

        joint_name = self.marker_to_joint[marker_name]
        cfg = self.joint_configs[joint_name]

        # Linear motion along X axis
        x = feedback.pose.position.x

        m_min = cfg["marker_min"]
        m_max = cfg["marker_max"]

        if x < m_min:
            x = m_min
        elif x > m_max:
            x = m_max

        j_low = cfg["joint_lower"]
        j_up = cfg["joint_upper"]

        if m_max != m_min:
            t = (x - m_min) / (m_max - m_min)
        else:
            t = 0.5  # fallback

        joint_angle = j_low + t * (j_up - j_low)

        # Update local joint position for this joint
        self.joint_positions[joint_name] = joint_angle

        # Build command message in the controller's joint order
        cmd = Float64MultiArray()
        cmd.data = [self.joint_positions[j] for j in self.joint_order]

        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"[{joint_name}] x={x:.3f} -> angle={joint_angle:.3f} rad"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExcavatorJointInteractive()
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

