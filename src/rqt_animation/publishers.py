import copy
import rospy
import sys

# animation framework
from expressive_motion_generation.animation_execution import Animation

# moveit
import moveit_commander
from moveit_commander.robot import RobotCommander

# messages
from moveit_msgs.msg import DisplayRobotState, DisplayTrajectory, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

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
        self.pub_trajectory = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
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
    
    def shutdown(self):
        self.pub_fake.unregister()
        self.pub_real.unregister()
