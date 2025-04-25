
# ROS
import os
import time
import sys
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

# RQT
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMenu, QFileDialog, QInputDialog, QMessageBox

# moveit
import moveit_commander
from moveit_commander.robot import RobotCommander

# custom widgets
from rqt_animation.plot_canvas import MplCanvas
from rqt_animation.dialog_scale import ScaleDialog
from rqt_animation.dialog_new import NewDialog
from rqt_animation.dialog_settings import SettingsDialog

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

        # publisher manager and topic names
        self.publishers = None
        self.topic_fake = '/display_robot_state'
        self.topic_command = '/joint_command'

        # clock subscriptor
        self.clock_sub = None

        # is the animation playing?
        self._playing = False

        # what was the timestamp of the last clock tick?
        self._last_time = -1.0

        # how fast should the animation be played
        self.playback_speed = 1.0

        # how long is the animation in the editor?
        self._animation_length = 0.0

        # is this a new animation file that has not been saved yet?
        self._new_file = False

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
        self.newButton.triggered.connect(self._on_newButton_clicked)
        self.openButton.triggered.connect(self._on_openButton_clicked)
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
        self._widget.deleteButton.clicked.connect(self._on_deleteButton_clicked)

        # trim and extend
        self._widget.trimButton.clicked.connect(self._on_trimButton_clicked)
        self._widget.extendButton.clicked.connect(self._on_extendButton_clicked)

        # playback speed
        self._widget.speedSlider.valueChanged.connect(self._on_speedSlider_valueChanged)
        self._widget.speedSpinBox.valueChanged.connect(self._on_speedSpinBox_valueChanged)

        # advance bezier editing mode
        self._widget.curveButton.clicked.connect(self._on_bezierButton_clicked)

    def shutdown_plugin(self):
        self.unload_animation()
        return super().shutdown_plugin()
    
    def save_settings(self, plugin_settings, instance_settings):
        # TODO: save config here
        # usually per instance_settings.set_value(k, v)
        instance_settings.set_value('file', self.animation_file)
        instance_settings.set_value('topic_fake', self.topic_fake)
        if self.topic_fake == '':
            self.topic_fake = '/display_robot_state'
        instance_settings.set_value('topic_command', self.topic_command)
        if self.topic_command == '':
            self.topic_command = '/joint_command'
        return super().save_settings(plugin_settings, instance_settings)
    
    def restore_settings(self, plugin_settings, instance_settings):
        #TODO: restore config
        # usually per v = instance_settings.value(k)
        ret = super().restore_settings(plugin_settings, instance_settings)
        if not instance_settings.value('file') is None:
            self._open_file(instance_settings.value('file'))
        self.topic_command = instance_settings.value('topic_command')
        self.topic_fake = instance_settings.value('topic_fake')
        return ret
    
    def trigger_configuration(self):
        """
        Open Settings
        """
        dialog = SettingsDialog(self.topic_command, self.topic_fake)
        result = dialog.exec()

        # if OK was pressed, save new topic names
        if result:
            self.topic_command = dialog._widget.jointCommandPublishEdit.text()
            self.topic_fake = dialog._widget.fakeJointStatePublishTopic.text()

            # if an animation is currently loaded, reload publishers
            if self.animation is not None:
                self.publishers.shutdown()
                self.publishers = PublisherManager(self.animation.move_group, self.topic_fake, self.topic_command)
    
    def unload_animation(self):
        """
        Unloads current animation and unregisters handlers
        """
        self.animation = None

        if not self.clock_sub is None:
            self.clock_sub.unregister()
            self.clock_sub = None

        if not self.publishers is None:
            self.publishers.shutdown()
            self.publishers = None
        
        self.plot.unload()
        
        self._widget.addButton.setEnabled(False)
        self._widget.deleteButton.setEnabled(False)
        self._widget.trimButton.setEnabled(False)
        self._widget.extendButton.setEnabled(False)
        self.saveButton.setEnabled(False)
        self.saveAsButton.setEnabled(False)

    def _new_animation(self):
        """
        Create a new animation
        """  
        # load moveit robot commander
        robot = None
        if not self.publishers is None:
            robot = self.publishers.robot
        else:
            moveit_commander.roscpp_initialize(sys.argv)
            robot = RobotCommander()
        
        # open new animation dialog
        print(robot.get_planning_frame())
        dialog = NewDialog(robot)
        result = dialog.exec()

        # check result
        if result:

            # unload old animation
            self.unload_animation()
            self._new_animation = True

            # get info from dialog
            name = dialog._widget.nameEdit.text()
            move_group = dialog._widget.moveGroupBox.currentText()

            # create new animation
            self.animation = Animation(None)
            self.animation.name = name
            self.animation.move_group = move_group
            self.animation.frame_id = robot.get_planning_frame()
            self.animation.joint_names = robot.get_active_joint_names()

            # initialize editor
            self._animation_length = 5.0
            self.publishers = PublisherManager(self.animation.move_group)
            self.publishers.publish_real_states = self._widget.publishCheckBox.checkState()

            # add current position as first keyframe
            # get robot state
            state = self.publishers.get_robot_state(self.animation.joint_names)

            # add keyframe
            self.animation.times = np.array([0.0])
            self.animation.positions = np.array([state])
            
            # draw plot for animation
            self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers)
            self.plot.set_xrange(0.0, self._animation_length)
            self.plot.draw_timebars(0.0)

            # configure time slider
            self._configure_time_slider()

            # enable buttons
            self._widget.addButton.setEnabled(True)
            self._widget.deleteButton.setEnabled(True)
            self._widget.trimButton.setEnabled(True)
            self._widget.extendButton.setEnabled(True)
            self.saveButton.setEnabled(True)
            self.saveAsButton.setEnabled(True)

            # subscribe to clock
            self.clock_sub = rospy.Subscriber("/clock", Clock, self._on_clock_tick, queue_size=10)

    def _open_file(self, file):
        """
        Load the animation from the specified file and configure all the UI
        elements accordingly
        """

        # if an animation was loaded previously, remove old handlers first
        self.unload_animation()

        self.animation_file = file

        # load animation
        self.animation = Animation(self.animation_file)
        self._animation_length = self.animation.times[-1]

        # initialize publishers
        try:
            self.publishers = PublisherManager(self.animation.move_group, self.topic_fake, self.topic_command)
            self.publishers.publish_real_states = self._widget.publishCheckBox.checkState()
        except ValueError:
            # if the move group was not found, cancel loading this animation
            dialog = QMessageBox(self._widget)
            dialog.setWindowTitle('Error')
            dialog.setText(f'Robot could not be loaded: Move Group {self.animation.move_group} was not available.')
            dialog.setStandardButtons(QMessageBox.Retry | QMessageBox.Cancel)
            dialog.setIcon(QMessageBox.Critical)
            result = dialog.exec()

            if result == QMessageBox.Retry:
                self._open_file(self.animation_file)
                return
            else:
                self.unload_animation()
                return

        # check if animation is suitable for currently loaded robot joints from MoveIt
        if not self.publishers.check_compatibility(self.animation):
            dialog = QMessageBox(self._widget)
            dialog.setWindowTitle('Error')
            dialog.setText(f'Robot could not be loaded: Joint names could not be matched.')
            dialog.setStandardButtons(QMessageBox.Retry | QMessageBox.Cancel)
            dialog.setIcon(QMessageBox.Critical)
            result = dialog.exec()

            if result == QMessageBox.Retry:
                self._open_file(self.animation_file)
                return
            else:
                self.unload_animation()
                return

        # draw plot for animation
        self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers)
        self.apply_joint_limits()
        self.plot.draw_timebars(0.0)

        # configure time slider
        self._configure_time_slider()

        # enable buttons
        self._widget.addButton.setEnabled(True)
        self._widget.deleteButton.setEnabled(True)
        self._widget.trimButton.setEnabled(True)
        self._widget.extendButton.setEnabled(True)
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
        self._widget.timeSlider.setRange(0, int(self._animation_length * 1000))

        # configure size (margins are 9px at every side)
        left, right = self.plot.get_bounds()

        # add additional space without keyframes
        if not self.animation.times[-1] == 0.0:
            right += (self._animation_length - self.animation.times[-1]) * ((right - left) / self.animation.times[-1])
        else:
            # there is only one frame in the animation, set the time slider to span from left to right
            left = 18
            right = self._widget.size().width() - 9

        self._widget.timeSlider.setMaximumSize(right - left, 100)
        self._widget.horizontalLayout.setContentsMargins(left - 9, 0, self._widget.size().width() - right - 18, 0)
    
    def _publish_planned_joint_state(self):
        
        # construct joint state message
        state = JointState()
        
        state.name = self.animation.joint_names

        # check if state from animation or from plot needs to be published
        override_positions = self.plot.get_override_joint_state()

        if override_positions is None:
            # get joint positions from animation
            time = self._widget.timeSlider.value() / 1000.0
            if time >= self.animation.times[-1]:
                state.position = self.animation.positions[-1].tolist()
            else:
                state.position = self.animation.trajectory_planner.get_position_at(time).tolist()
        
        else:
            # get joint positions from plot
            state.position = override_positions
        
        # publish
        self.publishers.publish_state(state)
        
    def update_animation_from_plot(self):
        """
        Called by plot widget.
        Applies changes from the plot to the animation data.
        """
        # reload animation trajectory
        self.animation._reload_trajectory()

        # redraw animation in case bezier curves were changed
        self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers, False)
        self._configure_time_slider()
        self._on_timeSlider_valueChanged()

    def save_animation(self, file_path):
        """
        Save the currently loaded animation in a file with the given path
        """
        file = open(file_path, mode='w')
        self.animation.save_yaml(file)
        file.close()

    def apply_joint_limits(self):
        """
        Queries joint position limits from robot commander and
        sends them to the plot widget
        """
        # get joint limits
        limits = []

        for name in self.animation.joint_names:
            try:
                joint = self.publishers.robot.get_joint(name)
                limits.append([joint.min_bound(), joint.max_bound()])
            except:
                print('ERROR: Joint', name, 'not found in robot.')

        # apply to plot
        self.plot.set_joint_limits(limits)


    # --------------------------------- BUTTON HANDLERS ----------------------------------

    def _on_newButton_clicked(self):
        """
        Init a new animation file and open it
        """
        self._new_animation()

    def _on_openButton_clicked(self):
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
        if self._new_file:
            return self._on_saveAsButton_clicked()
            
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
            if np.floor(time * 1000) > self._widget.timeSlider.value():
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
        self._configure_time_slider()
        self._on_timeSlider_valueChanged()

    def _on_deleteButton_clicked(self):
        '''
        Delete the currently selected keyframes
        '''
        if not self.animation is None:
            self.plot.remove_selection()
            self.animation.times = self.plot.times
            self.animation.positions = self.plot.positions
            self.animation.beziers = self.plot.beziers

            if not self.plot.bezier_mode:
                # reload plot
                # draw plot for animation
                self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers)
                self._configure_time_slider()
                self._on_timeSlider_valueChanged()

                self.animation._reload_trajectory()

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
                
                # reload trajectory
                self.animation.times = self.animation.trajectory_planner.times
                self.animation.positions = self.animation.trajectory_planner.positions
                self.animation._reload_trajectory()

                # redraw plot
                self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers)
                self.plot.draw_timebars(0.0)

                # configure time slider
                self._configure_time_slider()

    def _on_extendButton_clicked(self):
        """
        Open dialog to ask for extension length, then extend available plot space
        by the given amount
        """

        # open dialog to ask for amount
        amount, ok = QInputDialog.getDouble(self._widget, 'Extend animation length', 'How many seconds should the animation length be extended by?',
                                            1.0, 0.0, 1000.0, 3)
        
        if ok and amount:
            self._animation_length += amount
            self._configure_time_slider()
            self.plot.set_xrange(0, self._animation_length)

    def _on_trimButton_clicked(self):
        """
        Trim animation length to the last keyframe timestamp and plot
        y limits to fit everything
        """
        self._animation_length = self.animation.times[-1]
        self.plot.set_xrange(0, self._animation_length)
        self.plot.adjust_yrange()
        self._configure_time_slider()

    def _on_bezierButton_clicked(self):
        """
        Toggle advanced bezier curve editing mode
        """
        self.plot.toggle_bezier_mode()

        # redraw animation
        self.plot.load_animation(self.animation.positions, self.animation.times, self.animation.beziers)

        if self.plot.bezier_mode:
            self.plot.load_advanced_beziers()
            self._widget.curveButton.setText('Toggle Normal Animation Editing')

            # deactivate buttons
            self._widget.addButton.setEnabled(False)
            self._widget.trimButton.setEnabled(False)
            self._widget.extendButton.setEnabled(False)
        else:
            self.plot.unload_advanced_beziers()
            self._widget.curveButton.setText('Toggle Advanced Curve Editing')

            # activate buttons
            self._widget.addButton.setEnabled(True)
            self._widget.trimButton.setEnabled(True)
            self._widget.extendButton.setEnabled(True)
        
        self._configure_time_slider()
        self._on_timeSlider_valueChanged()

    def _on_speedSlider_valueChanged(self):
        """
        Adjust playback speed
        """
        value = (self._widget.speedSlider.value() + 50) / 50.0
        self._widget.speedSpinBox.valueChanged.disconnect()
        self._widget.speedSpinBox.setValue(value)
        self._widget.speedSpinBox.valueChanged.connect(self._on_speedSpinBox_valueChanged)
        self.playback_speed = self._widget.speedSpinBox.value()
    
    def _on_speedSpinBox_valueChanged(self):
        """
        Adjust speed slider
        """
        value = 50 * self._widget.speedSpinBox.value() - 50
        self._widget.speedSlider.valueChanged.disconnect()
        self._widget.speedSlider.setValue(int(value))
        self._widget.speedSlider.valueChanged.connect(self._on_speedSlider_valueChanged)
        self.playback_speed = self._widget.speedSpinBox.value()

    # ---------------------------------- ROS CALLBACK -----------------------------------

    def _on_clock_tick(self, clock: Clock):
        '''
        If playing the animation on real states, update time slider
        and send joint states
        '''
        if self._playing:

            #current_time = clock.clock.secs + (clock.clock.nsecs / 1000000000.0)
            current_time = time.time()

            # check if animation has already been initialized
            if self._last_time < 0:
                self._last_time = current_time

            time_diff = (current_time - self._last_time) * self.playback_speed

            # if time difference is too small, skip until it is big enough
            if time_diff < 0.001:
                return

            # check if end was reached
            if self._widget.timeSlider.maximum() <= self._widget.timeSlider.value() + (time_diff * 1000):
                # set state to last frame
                self._widget.timeSlider.setValue(self._widget.timeSlider.maximum())
                self._on_stopButton_clicked()
            
            # otherwise publish current joint state
            else:
                # current_time += time_diff
                self._last_time = current_time

                # set value on time slider
                # this will conveniently call the slider valueChanged signal and thus publish the joint state
                self._widget.timeSlider.setValue(self._widget.timeSlider.value() + int(time_diff * 1000))
        
        else:
            #
            self._configure_time_slider()

        # always publish robot state
        self._publish_planned_joint_state()
