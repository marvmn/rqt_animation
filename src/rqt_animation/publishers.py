import rospy

# messages
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState

class PublisherManager():

    def __init__(self):
        """
        Initialize publishers
        """

        # status variable
        self.publish_real = False

        # publishers
        self.pub_fake = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
        self.pub_real = rospy.Publisher("/joint_command", JointState, queue_size=10)