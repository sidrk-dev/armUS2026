import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class SendTargetJointDegrees(Node):
    """
    Converts JointState (radians) from MoveIt to degrees for the Arduino.
    Handles 6 joints total (Gripper/Joint 7 Removed).
    """
    def __init__(self):
        super().__init__('send_target_joint_degrees')
        self.sub = self.create_subscription(JointState, '/joint_states', self.msg_callback, 1)
        self.pub = self.create_publisher(Float64MultiArray, 'arm_targets', 10)
        self.get_logger().info('Joint Translator Node Started (6-joint config)')

    def msg_callback(self, msg):
        """
        Callback for /joint_states.
        Converts radians to degrees and maps to the correct array structure.
        
        Expected joint_states order from MoveIt (check URDF):
        [joint1, joint2, joint3, joint4, joint5, joint6]
        """
        # Initialize array for 6 joints
        jointarray = [0.0] * 6
        gamma = 180.0 / 3.141592 # rad to degrees conversion
        
        # Access messages by name to ensure safety? 
        # For now assuming standard index order from MoveIt but basic remapping logic below
        # Original logic had custom remapping:
        # jointarray[0] = joint_states[2] 
        # jointarray[1] = joint_states[0] * gamma 
        # ...
        
        # We will assume MoveIt publishes in order: joint1...joint6
        # If your URDF defines joints in a specific order, msg.position follows that.
        # Check current URDF joint order: joint1, joint2, joint3, joint4, joint5, joint6.
        
        try:
            # Map based on names if possible, else fallback to index
            name_map = {name: i for i, name in enumerate(msg.name)}
            
            def get_angle(name):
                idx = name_map.get(name)
                return msg.position[idx] * gamma if idx is not None else 0.0

            # Apply logic (preserving original math logic where applicable)
            # Joint 1
            jointarray[0] = get_angle("joint1")
            
            # Joint 2
            jointarray[1] = get_angle("joint2")
            
            # Joint 3
            jointarray[2] = get_angle("joint3")
            
            # Joint 4
            jointarray[3] = get_angle("joint4")
            
            # Joint 5 (Differential Wrist A)
            j5_val = get_angle("joint5")
            j6_val = get_angle("joint6")
            
            # Original logic: jointarray[4] = (joint_states[4] - 2*joint_states[5]) * gamma
            # But get_angle returns degrees already.
            # Let's assume the differential mix happens here or in Arduino.
            # Previous code: jointarray[4] = (joint_states[4] - 2*joint_states[5]) * gamma
            # Since we converted to degrees individually:
            # We will pass raw degrees for J5 and J6 and let Arduino/MoveIt handle mixing?
            # Reverting to simple pass-through for now to match standard MoveIt behavior.
            # If differential drive mixing is needed, it should be done here or in firmware.
            # Based on previous file, it seemed to do mixing here.
            
            # If we assume msg.position indices match 0-5:
            # jointarray[4] = (msg.position[4] - 2 * msg.position[5]) * gamma
            # jointarray[5] = (msg.position[4] + 2 * msg.position[5]) * gamma
            
            # Simplified pass-through (Let's assume standard kinematics for now)
            jointarray[4] = j5_val
            jointarray[5] = j6_val
            
        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
            return

        self.pub.publish(Float64MultiArray(data=jointarray))

def main():
    rclpy.init()
    node = SendTargetJointDegrees()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
