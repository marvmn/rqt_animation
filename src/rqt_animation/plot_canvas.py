# Matplotlib
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
    
    def load_animation(self, positions, times, beziers):
        '''
        Plot animation file
        '''
        self.lines = []
        self.scatters = []

        for i in range(len(positions.T)):
            l, = self.axes.plot(times, positions.T[i], linewidth=1)
            self.lines.append(l)
            s = self.axes.scatter(times, positions.T[i], marker=".")
            self.scatters.append(s)
