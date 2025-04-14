
# ROS
import os
import sys
import rospy
import rospkg
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

# RQT
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMenu, QFileDialog
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog

# custom widgets
from rqt_animation.plot_canvas import MplCanvas
from rqt_animation.dialog_scale import ScaleDialog

# Animation stuff
from expressive_motion_generation.animation_execution import Animation

from rqt_animation.publishers import PublisherManager

class QResizableWidget(QWidget):

    def __init__(self, on_resize):
        """
        Inits a QWidget that calls the given on_resize function when it is resized.
        """
        QWidget.__init__(self)
        self._on_resize = on_resize

    def resizeEvent(self, event):
        QWidget.resizeEvent(self, event)
        self._on_resize()

class AnimationEditor(Plugin):

    def __init__(self, context):
        super(AnimationEditor, self).__init__(context)

        # initialize member variables

        # animation filename
        # this is also used as standard directory for the file choosers
        self.animation_file = '/home'

        # animation instance
        self.animation = None

        # is the animation playing?
        self._playing = False

        # what was the timestamp of the last clock tick?
        self._last_time = -1.0

        # set name
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
        self._widget = QResizableWidget(self._configure_time_slider)

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
        color = self._widget.palette().window().color().getRgbF()
        self.plot = MplCanvas(self.update_animation_from_plot, color, self)
        self._widget.verticalLayout.insertWidget(4, self.plot)

        # create menus
        fileMenu = QMenu('File')
        self.newButton = fileMenu.addAction('New...')
        self.openButton = fileMenu.addAction('Open...')
        self.saveButton = fileMenu.addAction('Save')
        self.saveAsButton = fileMenu.addAction('Save as...')
        self.exitButton = fileMenu.addAction('Exit')
        self._widget.fileMenuButton.setMenu(fileMenu)

        editMenu = QMenu('Edit')
        self.scaleButton = editMenu.addAction('Scale Animation...')
        self._widget.editMenuButton.setMenu(editMenu)

        # connect actions
        self._connect_actions()

    def _connect_actions(self):
        '''
        Connect all buttons etc. with actions
        '''

        # file menu
        self.openButton.triggered.connect(self._on_fileButton_clicked)
        self.saveButton.triggered.connect(self._on_saveButton_clicked)
        self.saveAsButton.triggered.connect(self._on_saveAsButton_clicked)
        self.exitButton.triggered.connect(self._on_exitButton_clicked)

        # edit menu
        self.scaleButton.triggered.connect(self._on_scaleButton_clicked)

        # time slider
        self._widget.timeSlider.valueChanged.connect(self._on_timeSlider_valueChanged)

        # state changed
        self._widget.publishCheckBox.stateChanged.connect(self._on_publish_checkBox_changed)

        # play and stop buttons
        self._widget.playButton.clicked.connect(self._on_playButton_clicked)
        self._widget.stopButton.clicked.connect(self._on_stopButton_clicked)

        # next and previous buttons
        self._widget.nextButton.clicked.connect(self._on_nextButton_clicked)
        self._widget.previousButton.clicked.connect(self._on_previousButton_clicked)

        # last and first buttons
        self._widget.firstButton.clicked.connect(self._on_firstButton_clicked)
        self._widget.lastButton.clicked.connect(self._on_lastButton_clicked)

        # add and delete keyframe buttons
        self._widget.addButton.clicked.connect(self._on_addButton_clicked)

    def shutdown_plugin(self):
        self.publishers.shutdown()
        self.clock_sub.unregister()
        return super().shutdown_plugin()
    
    def save_settings(self, plugin_settings, instance_settings):
        # TODO: save config here
        # usually per instance_settings.set_value(k, v)
        instance_settings.set_value('file', self.animation_file)
        return super().save_settings(plugin_settings, instance_settings)
    
    def restore_settings(self, plugin_settings, instance_settings):
        #TODO: restore config
        # usually per v = instance_settings.value(k)
        ret = super().restore_settings(plugin_settings, instance_settings)
        if not instance_settings.value('file') is None:
            self._open_file(instance_settings.value('file'))
        return ret
    
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
        self.publishers.publish_real_states = self._widget.publishCheckBox.checkState()

        # load button now contains the animation name
        #self._widget.fileButton.setText(self.animation.name + " (Open other file...)")

        # enable save option
        self.saveButton.setEnabled(True)
        self.saveAsButton.setEnabled(True)

        # subscribe to clock
        self.clock_sub = rospy.Subscriber("/clock", Clock, self._on_clock_tick, queue_size=10)

    def _configure_time_slider(self):
        """
        Sets value range, layout position and size on screen of the time slider
        """

        # if animation has not been set yet, do nothing
        if self.animation is None:
            return

        # configure range: Convert to ms because QSlider only works with integers
        self._widget.timeSlider.setRange(0, int(self.animation.times[-1] * 1000))

        # configure size (margins are 9px at every side)
        left, right = self.plot.get_bounds()
        self._widget.timeSlider.setMaximumSize(right - left, 100)
        self._widget.horizontalLayout.setContentsMargins(left - 9, 0, self._widget.size().width() - right - 18, 0)
    
    def _publish_planned_joint_state(self):
        
        # construct joint state message
        state = JointState()
        
        state.name = self.animation.joint_names
        state.position = self.animation.trajectory_planner.get_position_at(self._widget.timeSlider.value() / 1000.0).tolist()
        
        # publish
        self.publishers.publish_state(state)
        
    def update_animation_from_plot(self):
        """
        Called by plot widget.
        Applies changes from the plot to the animation data.
        """
        self.animation._reload_trajectory()

    def save_animation(self, file_path):
        """
        Save the currently loaded animation in a file with the given path
        """
        file = open(file_path, mode='w')
        self.animation.save_yaml(file)
        file.close()


    # --------------------------------- BUTTON HANDLERS ----------------------------------

    def _on_fileButton_clicked(self):
        """
        Open an animation file
        """
        file = QFileDialog.getOpenFileName(self._widget, "Open File", self.animation_file, 'Animation YAML Files (*.yaml)')

        # check if a file was chosen
        if not file[0] == '':
            self._open_file(file[0])
    
    def _on_saveButton_clicked(self):
        """
        Save current animation in the same file as before
        """
        self.save_animation(self.animation_file)
    
    def _on_saveAsButton_clicked(self):
        """
        Save current animation as a new file and set it as the current file
        """
        file = QFileDialog.getSaveFileName(self._widget, "Save File", self.animation_file, 'Animation YAML Files (*.yaml)')

        # check if a file was chosen
        if not file[0] == '':
            self.animation_file = file[0]
            self.save_animation(file[0])

    def _on_exitButton_clicked(self):
        """
        Terminate the program
        """
        self.shutdown_plugin()
        sys.exit()

    def _on_timeSlider_valueChanged(self):
        self.plot.draw_timebars(self._widget.timeSlider.value() / 1000.0)
    
    def _on_publish_checkBox_changed(self):
        self.publishers.publish_real_states = self._widget.publishCheckBox.checkState()

    def _on_playButton_clicked(self):
        """
        Play the animation from the selected time until the end, either as 
        DisplayTrajectory or on the real robot depending on the setting
        """
        self._playing = True
        self._widget.playButton.setEnabled(False)
        self._widget.stopButton.setEnabled(True)
    
    def _on_stopButton_clicked(self):
        """
        Stop the animation
        """
        self._playing = False
        self._last_time = -1.0
        self._widget.playButton.setEnabled(True)
        self._widget.stopButton.setEnabled(False)
    
    def _on_nextButton_clicked(self):
        """
        Search next keyframe and set time slider to it's time
        """
        for time in self.animation.times:

            # round time to 3 digits after comma to avoid rounding errors
            # since the time slider works with millisecond integers, a
            # maximum of 3 digits makes sense here.
            if round(time, 3) > self._widget.timeSlider.value() / 1000.0:
                self._widget.timeSlider.setValue(time * 1000.0)
                return
    
    def _on_previousButton_clicked(self):
        """
        Search previous keyframe and set time slider to it's time
        """
        for time in list(reversed(self.animation.times)):

            # round time to 3 digits after comma to avoid rounding errors
            # since the time slider works with millisecond integers, a
            # maximum of 3 digits makes sense here.
            if round(time, 3) < self._widget.timeSlider.value() / 1000.0:
                self._widget.timeSlider.setValue(time * 1000.0)
                return
    
    def _on_lastButton_clicked(self):
        """
        Jump to the last keyframe
        """
        self._widget.timeSlider.setValue(self.animation.times[-1] * 1000.0)
    
    def _on_firstButton_clicked(self):
        """
        Jump to the first keyframe
        """
        self._widget.timeSlider.setValue(self.animation.times[0] * 1000.0)
    
    def _on_addButton_clicked(self):
        """
        Check the current robot state and set it as keyframe for the
        selected time
        """
        # get robot state
        state = self.publishers.get_robot_state(self.animation.joint_names)

        # add keyframe
        self.animation.add_keyframe(self._widget.timeSlider.value() / 1000.0, state)

        # reload plot
        # draw plot for animation
        self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers)
        self._on_timeSlider_valueChanged()

    def _on_scaleButton_clicked(self):
        """
        Open scale animation dialog
        """
        
        # check if animation is opened
        if not self.animation is None:

            # get animation length
            length = self.animation.times[-1]

            # open dialog
            dialog = ScaleDialog(length)
            result = dialog.exec()

            # check result
            if result:

                # get scalar value
                factor = dialog._widget.factorSpinBox.value()

                # apply it to animation
                self.animation.trajectory_planner.times = self.animation.times
                self.animation.trajectory_planner.positions = self.animation.positions
                self.animation.trajectory_planner.scale_global_speed(factor)

                # redraw plot
                self.plot.load_animation(self.animation.trajectory_planner.positions, 
                                         self.animation.trajectory_planner.times, 
                                         self.animation.beziers)
                
                # reload trajectory
                self.animation.times = self.animation.trajectory_planner.times
                self.animation.positions = self.animation.trajectory_planner.positions
                self.animation._reload_trajectory()

            

    # ---------------------------------- ROS CALLBACK -----------------------------------

    def _on_clock_tick(self, clock: Clock):
        '''
        If playing the animation on real states, update time slider
        and send joint states
        '''
        if self._playing:

            current_time = clock.clock.secs + (clock.clock.nsecs / 1000000000.0)

            # check if animation has already been initialized
            if self._last_time < 0:
                self._last_time = current_time

            time_diff = current_time - self._last_time

            # check if end was reached
            if self._widget.timeSlider.maximum() <= self._widget.timeSlider.value() + (time_diff * 5000):
                # set state to last frame
                self._widget.timeSlider.setValue(self._widget.timeSlider.maximum())
                self._on_stopButton_clicked()
            
            # otherwise publish current joint state
            else:
                current_time += time_diff
                self._last_time = current_time

                # set value on time slider
                # this will conveniently call the slider valueChanged signal and thus publish the joint state
                self._widget.timeSlider.setValue(self._widget.timeSlider.value() + int(time_diff * 5000))
        
        else:
            #
            self._configure_time_slider()

        # always publish robot state
        self._publish_planned_joint_state()
