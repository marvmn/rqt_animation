# default pkgs
import os

# ROS
import rospkg

# QT bindings
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog

class ScaleDialog(QDialog):

    def __init__(self, current_length):

        super().__init__()

        self._original_length = current_length

        # initialize content
        self._widget = QWidget()

        # load UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_animation'), 'resources', 'scale_dialog.ui')
        self._widget.setObjectName('ScaleDialogUI')

        # add widgets from file to this widget
        loadUi(ui_file, self._widget)

        self.setLayout(self._widget.layout())

        # set values for the current length
        self._widget.titleLabel.setText(f"Scale animation from {current_length:.3f}s to...")
        self._widget.factorSpinBox.setValue(1.00)
        self._widget.secondsSpinBox.setValue(current_length)

        # connect actions
        self._connect_actions()
    
    def _connect_actions(self):
        """
        Connect widget signals to corresponding functions
        """
        self._widget.buttonBox.accepted.connect(self.accept)
        self._widget.buttonBox.rejected.connect(self.reject)
        self._widget.factorSpinBox.valueChanged.connect(self._on_factor_changed)
        self._widget.secondsSpinBox.valueChanged.connect(self._on_seconds_changed)

    def _on_factor_changed(self, value):
        '''
        Called when the spin box for the scaling factor is changed
        '''
        self._widget.secondsSpinBox.setValue(self._original_length * value)

    def _on_seconds_changed(self, value):
        '''
        Called when the spin box for the length in seconds is changed
        '''
        self._widget.factorSpinBox.setValue(value / self._original_length)