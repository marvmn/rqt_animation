# default pkgs
import os

# ROS
import rospkg

# QT bindings
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog

class NewDialog(QDialog):

    def __init__(self, robot):

        super().__init__()

        self.robot = robot

        # initialize content
        self._widget = QWidget()

        # load UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_animation'), 'resources', 'new_dialog.ui')
        self._widget.setObjectName('NewDialog')

        # add widgets from file to this widget
        loadUi(ui_file, self._widget)

        self.setLayout(self._widget.layout())
        self.setWindowTitle('New Animation')

        # init move group settings
        self._widget.moveGroupBox.addItems(robot.get_group_names())

        # connect actions
        self._connect_actions()
    
    def _connect_actions(self):
        """
        Connect widget signals to corresponding functions
        """
        self._widget.buttonBox.accepted.connect(self.accept)
        self._widget.buttonBox.rejected.connect(self.reject)