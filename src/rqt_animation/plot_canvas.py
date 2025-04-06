# Matplotlib
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.fig.tight_layout()

        # list of vertical lines that mark the positions of keyframes
        self.timebars = None

        # vertical line that indicates which position is currently selected
        self.indicator = None

        super().__init__(self.fig)
    
    def load_animation(self, positions, times, beziers):
        '''
        Plot animation file
        '''
        # save information
        self.positions = positions
        self.times = times
        self.beziers = beziers

        # draw animation lines and points
        self.lines = []
        self.scatters = []

        for i in range(len(positions.T)):
            l, = self.axes.plot(times, positions.T[i], linewidth=1)
            self.lines.append(l)
            s = self.axes.scatter(times, positions.T[i], marker=".")
            self.scatters.append(s)
        
        # adjust plot size
        self.fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.15)
        
    
    def draw_timebars(self, time):
        """
        Draw vertical lines that mark the positions of keyframes and a distinct line
        that indicates which time is currently selected
        """
        # first delete old lines
        if not self.timebars is None:
            self.timebars.remove()
        if not self.indicator is None:
            self.indicator.remove()
        
        # now draw vlines
        self.timebars = self.axes.vlines(self.times, -4, 4, linestyles="dotted")
        self.indicator = self.axes.vlines(time, -5, 5, linestyles='dashed')

        # refresh
        self.draw()
