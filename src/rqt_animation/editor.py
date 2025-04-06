
# ROS
import os
import rospy
import rospkg
from sensor_msgs.msg import JointState

# RQT
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog
from python_qt_binding.QtWidgets import QFileDialog

# custom widget
from rqt_animation.plot_canvas import MplCanvas

# Animation stuff
from expressive_motion_generation.animation_execution import Animation

from rqt_animation.publishers import PublisherManager

class AnimationEditor(Plugin):

    def __init__(self, context):
        super(AnimationEditor, self).__init__(context)

        self.animation_file = None
        self.animation = None

        self.setObjectName('AnimationEditor')

        # process standalone plugin command line args
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)
        
        # create QWidget
        self._widget = QWidget()

        # load UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_animation'), 'resources', 'animation_plugin.ui')

        self._widget.setObjectName('AnimationEditorUI')

        # add widgets from file to this widget
        loadUi(ui_file, self._widget)

        # set window title with a number if multiple plugins are opened
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # add to UI
        context.add_widget(self._widget)

        # add custom matplotlib widget
        self.plot = MplCanvas(self)
        self._widget.verticalLayout.insertWidget(4, self.plot)

        # connect actions
        self._connect_actions()

    def _connect_actions(self):
        '''
        Connect all buttons etc. with actions
        '''

        # file panel
        self._widget.fileButton.clicked.connect(self._on_fileButton_clicked)

        # time slider
        self._widget.timeSlider.valueChanged.connect(self._on_timeSlider_valueChanged)

    def shutdown_plugin(self):
        # TODO: unregister all publishers here
        return super().shutdown_plugin()
    
    def save_settings(self, plugin_settings, instance_settings):
        # TODO: save config here
        # usually per instance_settings.set_value(k, v)
        instance_settings.set_value('file', self.animation_file)
        return super().save_settings(plugin_settings, instance_settings)
    
    def restore_settings(self, plugin_settings, instance_settings):
        #TODO: restore config
        # usually per v = instance_settings.value(k)
        if not instance_settings.value('file') is None:
            self._open_file(instance_settings.value('file'))
        return super().restore_settings(plugin_settings, instance_settings)
    
    def trigger_configuration(self):
        """
        Open Settings
        """
        dialog = SimpleSettingsDialog(title='Animation Editor Settings')
        
        # TODO Hier Einstellungen falls notwendig
    
    def _open_file(self, file):
        """
        Load the animation from the specified file and configure all the UI
        elements accordingly
        """
        self.animation_file = file

        # load animation
        self.animation = Animation(self.animation_file)

        # draw plot for animation
        self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers)
        self.plot.draw_timebars(0.0)

        # configure time slider
        self._configure_time_slider()

        # initialize publishers
        self.publishers = PublisherManager(self.animation.move_group)
        self.publishers.publish_real_states = True

        # load button now contains the animation name
        self._widget.fileButton.setText(self.animation.name + " (Open other file...)")

        # enable save option
        self._widget.saveButton.setEnabled(True)

    def _configure_time_slider(self):
        
        # configure range: Convert to ms because QSlider only works with integers
        self._widget.timeSlider.setRange(0, int(self.animation.times[-1] * 1000))
    
    def _publish_planned_joint_state(self):
        
        # construct joint state message
        state = JointState()
        
        state.name = self.animation.joint_names
        state.position = self.animation.trajectory_planner.get_position_at(self._widget.timeSlider.value() / 1000.0).tolist()
        
        # publish
        self.publishers.publish_state(state)
        
    
    # --------------------------------- BUTTON HANDLERS ----------------------------------

    def _on_fileButton_clicked(self):
        file = QFileDialog.getOpenFileName(self._widget, "Open File", '/home', 'Animation YAML Files (*.yaml)')
        self._open_file(file[0])
    
    def _on_timeSlider_valueChanged(self):
        self.plot.draw_timebars(self._widget.timeSlider.value() / 1000.0)
        self._publish_planned_joint_state()
