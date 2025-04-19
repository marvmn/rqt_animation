# default pkgs
import os

# ROS
import rospkg

# QT bindings
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog

class SettingsDialog(QDialog):

    def __init__(self, joint_command_topic, fake_topic):

        super().__init__()

        # initialize content
        self._widget = QWidget()

        # load UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_animation'), 'resources', 'settings_dialog.ui')
        self._widget.setObjectName('SettingsDialog')

        # add widgets from file to this widget
        loadUi(ui_file, self._widget)

        self.setLayout(self._widget.layout())
        self.setWindowTitle('Settings')

        # initialize text fields
        self._widget.fakeJointStatePublishTopic.setText(fake_topic)
        self._widget.jointCommandPublishEdit.setText(joint_command_topic)

        # connect actions
        self._connect_actions()
    
    def _connect_actions(self):
        """
        Connect widget signals to corresponding functions
        """
        self._widget.buttonBox.accepted.connect(self.accept)
        self._widget.buttonBox.rejected.connect(self.reject)