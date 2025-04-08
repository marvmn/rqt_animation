import copy
import rospy
import numpy as np
import sys

# animation framework
from expressive_motion_generation.animation_execution import Animation

# moveit
import moveit_commander
from moveit_commander.robot import RobotCommander

# messages
from moveit_msgs.msg import DisplayRobotState
from sensor_msgs.msg import JointState

class PublisherManager():

    def __init__(self, group_name):
        """
        Initialize publishers and MoveIt!
        """
        # initialize moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # status variable
        self.publish_real_states = False

        # publishers
        self.pub_fake = rospy.Publisher("/display_robot_state", DisplayRobotState, queue_size=10)
        self.pub_real = rospy.Publisher("/joint_command", JointState, queue_size=10)

    def publish_state(self, state):
        """
        Publish a single joint state
        """
        if self.publish_real_states:
            self.pub_real.publish(state)
        # else:

        display = DisplayRobotState()
        display.state.joint_state = state
        self.pub_fake.publish(display)
    
    def get_robot_state(self, joint_names):
        """
        Returns the current robot joint state
        """

        # get robot state
        robotstate = self.robot.get_current_state()

        # initialize position array
        joint_positions = np.zeros(len(joint_names))

        # fill array with joint states in correct joint name order
        try:
            for i in range(len(joint_names)):
                idx = robotstate.joint_state.name.index(joint_names[i])
                joint_positions[i] = robotstate.joint_state.position[idx]
        except ValueError as e:
            print("ERROR: Mismatch between joint names!")
            print(e)
            return None
            
        # return finished array
        return joint_positions
    
    def shutdown(self):
        self.pub_fake.unregister()
        self.pub_real.unregister()
