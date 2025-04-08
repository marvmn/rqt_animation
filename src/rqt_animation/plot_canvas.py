# Matplotlib
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np
import copy

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.fig.tight_layout()

        # list of vertical lines that mark the positions of keyframes
        self.timebars = None

        # vertical line that indicates which position is currently selected
        self.indicator = None

        # currently selected points
        self.selected = []
        self.hovered = None

        # rectangle that appears when dragging the mouse to select something
        self.selection_rect = None

        super().__init__(self.fig)
    
    def load_animation(self, positions, times, beziers):
        '''
        Plot animation file
        '''
        # save information
        self.positions = positions
        self.times = times
        self.beziers = beziers

        # delete old plot if there is one
        self.fig.clear(False)
        self.axes = self.fig.add_subplot(111)

        # draw animation lines and points
        self.lines = []
        self.scatters = []

        sizes = np.ones(len(times)) * 36
        print(sizes)

        for i in range(len(positions.T)):

            # draw line
            l, = self.axes.plot(times, positions.T[i], linewidth=1)
            self.lines.append(l)

            # draw point (and make it pickable)
            s = self.axes.scatter(times, positions.T[i], marker=".", s=copy.deepcopy(sizes))
            s.set_picker(True)
            self.scatters.append(s)
        
        # adjust plot size
        self.fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.15)

        # connect events
        self.mpl_connect('button_press_event', self._on_mouse_press)
        self.mpl_connect('button_release_event', self._on_mouse_release)
        self.mpl_connect('motion_notify_event', self._on_mouse_move)
        self.mpl_connect('pick_event', self._on_point_picked)
    
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
    
    # ------------------- EVENT HANDLERS ---------------------

    def _on_mouse_press(self, event):
        """
        Called when the user presses a mouse button in the plot window
        """
        for s in self.scatters:
            hit, item_dict = s.contains(event)
            if hit:
                index = item_dict['ind'][0]
                item = s.get_paths()[item_dict['ind'][0]]
                print(item)

    def _on_mouse_release(self, event):
        """
        Called when the user releases a mouse button
        """
        print("release")

    def _on_mouse_move(self, event):
        """
        Called when the user moves their mouse
        """

        # check if a point was clicked
        for s in self.scatters:
            hit, item_dict = s.contains(event)

            # point found?
            if hit:

                # get point index
                index = item_dict['ind'][0]

                # if this point has already been marked as hovered, skip
                if self.hovered == (s, index):
                    return
                
                # deselect old point if there was one different from the current
                if not self.hovered is None:
                    self.hovered[0].get_sizes()[self.hovered[1]] = 36

                # mark as hovered and increase size of this point
                self.hovered = (s, index)
                sizes = s.get_sizes()
                sizes[index] *= 2
                s.set_sizes(sizes)

                # redraw canvas
                self.draw()
                return
        
        # if no point was touched, deselect and unhover
        if not self.hovered is None:
            self.hovered[0].get_sizes()[self.hovered[1]] = 36
            self.hovered = None
            self.draw()

    def _on_point_picked(self, event):
        """
        Called when a point is picked with the mouse
        """
        print("pick")
