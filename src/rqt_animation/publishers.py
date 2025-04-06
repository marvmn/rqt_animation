import copy
import rospy
import sys

# moveit
import moveit_commander
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import MoveGroupCommander

# messages
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
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
        self.pub_fake = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
        self.pub_real = rospy.Publisher("/joint_command", JointState, queue_size=10)

    def publish_state(self, state):
        """
        Publish a single joint state
        """
        if self.publish_real_states:
            self.pub_real.publish(state)
        # else:
        
        # create a fake trajectory that contains the single state needed
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.model_id = "panda"

        fake_trajectory = RobotTrajectory()
        fake_trajectory.joint_trajectory.header.frame_id = "panda_link0"
        fake_trajectory.joint_trajectory.joint_names = self.robot.get_active_joint_names()
        point = JointTrajectoryPoint()
        point.positions = state.position
        fake_trajectory.joint_trajectory.points.append(point)

        display_trajectory.trajectory.append(fake_trajectory)

        # publish!
        self.pub_fake.publish(display_trajectory)
