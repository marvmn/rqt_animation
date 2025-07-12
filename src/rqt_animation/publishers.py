import copy
import rospy
import numpy as np
import sys

# animation framework
from expressive_motion_generation.animation import Animation

# moveit
import moveit_commander
from moveit_commander.robot import RobotCommander

# messages
from moveit_msgs.msg import DisplayRobotState, ObjectColor
from sensor_msgs.msg import JointState

class PublisherManager():

    def __init__(self, group_name, fake_publish_topic="/display_robot_state", joint_command_topic="/joint_command"):
        """
        Initialize publishers and MoveIt!
        Raises a ValueError if group_name is not available.
        """
        # initialize moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()

        # check if move group is available
        if not group_name in self.robot.get_group_names():
            raise ValueError()

        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # status variables
        self.publish_real_states = False
        self.stopped = False

        # publishers
        self.pub_fake = rospy.Publisher(fake_publish_topic, DisplayRobotState, queue_size=10)
        self.pub_real = rospy.Publisher(joint_command_topic, JointState, queue_size=10)


    def publish_state(self, state, highlight=None):
        """
        Publish a single joint state.
        If highlight is not None, the given joint is highlighted in a color.
        """
        # first check, if the publisher is still registered
        # it's possilbe that it has already been unregistered, but due to
        # different threads for the GUI and ROS, this method is still being called
        if self.stopped:
            return

        # if real states should be published, use real publisher
        if self.publish_real_states:
            self.pub_real.publish(state)
        
        # otherwise use display robot state publisher
        else:
            display = DisplayRobotState()
            display.state.joint_state = state

            # if a link to highlight is given, add it to message
            if highlight is not None:
                object_color = ObjectColor()
                object_color.id = highlight
                object_color.color.g = 1.0
                object_color.color.r = 0.2
                object_color.color.b = 0.5
                object_color.color.a = 1.0
                display.highlight_links.append(object_color)

            # publish!
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

    def check_compatibility(self, animation):
        '''
        Checks if the given animation is applicable for the loaded robot.
        '''
        
        # check if joint names are in the move group
        if not set(animation.joint_names).issubset(self.move_group.get_active_joints()):
            return False
        
        # if all checks passed, this animation is compatible.
        return True
    
    def shutdown(self):
        self.stopped = True
        self.pub_fake.unregister()
        self.pub_real.unregister()
