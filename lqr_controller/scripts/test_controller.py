#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class ControllerTester:
    def __init__(self):
        rospy.init_node('controller_tester', anonymous=True)
        
        # Publishers
        self.desired_state_pub = rospy.Publisher('/desired_state', Float64MultiArray, queue_size=10)
        
        # Subscribers
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Variables
        self.current_state = np.zeros(6)
        self.state_initialized = False
        
        # Wait for joint states
        rospy.loginfo("Waiting for joint state messages...")
        rospy.sleep(2.0)
        
        if not self.state_initialized:
            rospy.logwarn("No joint state messages received. Continuing anyway.")
        
    def joint_state_callback(self, msg):
        # Update current state
        try:
            # Find indices for the joints we care about
            left_idx = msg.name.index('joint01_left')
            right_idx = msg.name.index('joint01_right')
            
            # Update current state (simplified)
            self.current_state[4] = (msg.position[left_idx] + msg.position[right_idx]) / 2.0  # phi
            self.current_state[5] = (msg.velocity[left_idx] + msg.velocity[right_idx]) / 2.0  # phi_dot
            
            self.state_initialized = True
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Error processing joint state: {e}")
    
    def run_test_stand_up(self):
        """Test the robot standing up to a desired height"""
        rospy.loginfo("Running stand-up test")
        
        # Create message
        msg = Float64MultiArray()
        
        # Set initial state (current state but with target height)
        target_state = np.copy(self.current_state)
        target_state[0] = 0.0  # theta (angle)
        target_state[1] = 0.0  # theta_dot (angular velocity)
        target_state[2] = 0.10  # l0 (height - slightly higher than default)
        target_state[3] = 0.0  # l0_dot (vertical velocity)
        
        msg.data = target_state.tolist()
        
        # Publish desired state
        self.desired_state_pub.publish(msg)
        rospy.loginfo(f"Published target state: {target_state}")
        
        # Wait for some time
        rospy.sleep(5.0)
        
        # Return to original height
        target_state[2] = 0.08  # default height
        msg.data = target_state.tolist()
        self.desired_state_pub.publish(msg)
        rospy.loginfo(f"Published target state: {target_state}")
    
    def run_test_balance(self):
        """Test the robot maintaining balance with different target angles"""
        rospy.loginfo("Running balance test")
        
        # Create message
        msg = Float64MultiArray()
        
        # Small forward tilt
        target_state = np.copy(self.current_state)
        target_state[0] = 0.1  # theta (angle) - small forward tilt
        target_state[1] = 0.0  # theta_dot
        target_state[2] = 0.08  # l0 (height)
        target_state[3] = 0.0  # l0_dot
        
        msg.data = target_state.tolist()
        self.desired_state_pub.publish(msg)
        rospy.loginfo(f"Published forward tilt: {target_state}")
        rospy.sleep(3.0)
        
        # Small backward tilt
        target_state[0] = -0.1  # theta (angle) - small backward tilt
        msg.data = target_state.tolist()
        self.desired_state_pub.publish(msg)
        rospy.loginfo(f"Published backward tilt: {target_state}")
        rospy.sleep(3.0)
        
        # Return to neutral
        target_state[0] = 0.0  # theta (angle)
        msg.data = target_state.tolist()
        self.desired_state_pub.publish(msg)
        rospy.loginfo(f"Published neutral angle: {target_state}")

def main():
    tester = ControllerTester()
    
    # Wait for everything to initialize
    rospy.sleep(1.0)
    
    # Run tests
    tester.run_test_stand_up()
    rospy.sleep(2.0)
    tester.run_test_balance()
    
    rospy.loginfo("Tests completed")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass