# Matplotlib
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
import numpy as np
import copy

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, update_callback, background, parent=None, width=10, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.fig.tight_layout(pad=0.0)
        self.fig.set_facecolor(background)
        self.axes.set_facecolor(background)
        self.update_callback = update_callback

        # default values
        self.default_color = np.array([0.1, 0.2, 0.6, 1.0])
        self.default_size = 64

        # list of vertical lines that mark the positions of keyframes
        self.timebars = None

        # vertical line that indicates which position is currently selected
        self.indicator = None

        # currently selected points
        self.selected = []
        self.hovered = None

        # if the mouse has been held down, save the point where it was first pressed
        self.grabbed = None

        # rectangle that appears when dragging the mouse to select something
        self.selection_rect = None

        # are these keys pressed currently?
        self.key_ctrl = False

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
        self.axes.set_facecolor(self.fig.get_facecolor())

        # draw animation lines and points
        self.lines = []
        self.scatters = []

        sizes = np.ones(len(times)) * self.default_size
        colors = np.tile(self.default_color, (len(times), 1))
        

        for i in range(len(positions.T)):

            # draw line
            l, = self.axes.plot(times, positions.T[i], linewidth=1)
            self.lines.append(l)

            # draw point (and make it pickable)
            s = self.axes.scatter(times, positions.T[i], marker=".", s=copy.deepcopy(sizes), c=copy.deepcopy(colors))
            s.set_picker(True)
            s.set_label("joint" + str(i))
            self.scatters.append(s)
        
        # adjust plot size
        self.fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.15)

        # connect events
        self.mpl_connect('button_press_event', self._on_mouse_press)
        self.mpl_connect('button_release_event', self._on_mouse_release)
        self.mpl_connect('motion_notify_event', self._on_mouse_move)
        self.mpl_connect('key_press_event', self._on_key_press)
        self.mpl_connect('key_release_event', self._on_key_release)
        self.mpl_connect('axes_enter_event', self.on_enter_event)
    
    def get_bounds(self):
        """
        Returns the x pixel coordinates of the left and right bounds of the plot
        """
        left_x = self.axes.transData.transform([self.times[0], 0])[0]
        right_x = self.axes.transData.transform([self.times[-1], 0])[0]
        return left_x, right_x

    def draw_timebars(self, time):
        """
        Draw vertical lines that mark the positions of keyframes and a distinct line
        that indicates which time is currently selected.
        If time is None, the indicator is not redrawn and stays the same.
        """
        # first delete old lines
        if not self.timebars is None:
            self.timebars.remove()
        if not self.indicator is None and not time is None:
            self.indicator.remove()
        
        # now draw vlines
        self.timebars = self.axes.vlines(self.times, -4, 4, linestyles="dotted")

        if not time is None:
            self.indicator = self.axes.vlines(time, -5, 5, linestyles='dashed')

        # refresh
        self.draw()
    
    # ------------------- EVENT HANDLERS ---------------------

    def _on_mouse_press(self, event):
        """
        Called when the user presses a mouse button in the plot window
        """

        # save the point where the mouse button was clicked
        self.grabbed = [event.xdata, event.ydata]
        
        # check if the mouse was hovering over a point and clicked it now
        if not self.hovered is None:
            
            # select point
            self.selected.append(self.hovered)

            # make the point a little bit bigger
            self.hovered[0].get_sizes()[self.hovered[1]] = self.default_size * 5

            # show selection
            self._update_selection()
        
        # if not, nothing was clicked on, so we deselect everything
        else:
            self.selected = []
            self._update_selection()
            

    def _on_mouse_release(self, event):
        """
        Called when the user releases a mouse button
        """
        
        # if there is a selection, update times and positions values according to
        # plot values (if they were moved while the mouse was held down)
        if self.selected:

            for (s, i) in self.selected:
                joint_idx = int(s.get_label()[len('joint'):])

                self.times[i] = s._offsets[i][0]
                self.positions[i][joint_idx] = s._offsets[i][1]
            
            # redraw time bars
            self.draw_timebars(None)

            # update animation data in parent widget
            self.update_callback()

        # check if a point is hovered and was just released
        if not self.hovered is None and self.hovered in self.selected:

            # deselect point
            self.selected.remove(self.hovered)

            # make point smaller again to default hover size
            self.hovered[0].get_sizes()[self.hovered[1]] = self.default_size * 2
            
            # visualize
            self._update_selection()
        
        # delete mouse grab point
        self.grabbed = None

        # remove selection rect
        if not self.selection_rect is None:
            self.selection_rect.remove()
            self.selection_rect = None
            self.draw()

    def _on_mouse_move(self, event):
        """
        Called when the user moves their mouse
        """

        # first check if something is selected and the mouse button has been pressed
        # while there is no selection rect.
        # That means that the current selection is being moved.
        if self.selected and not self.grabbed is None and self.selection_rect is None:

            # something is selected and the mouse was dragged
            # move selected points the specified amount
            movement = np.array([event.xdata, event.ydata]) - self.grabbed

            # if CTRL was held down, only move on the y axis
            if self.key_ctrl:
                movement[0] = 0.0

            for (s, i) in self.selected:
                joint_idx = int(s.get_label()[len('joint'):])
                original_position = np.array([self.times[i], self.positions[i][joint_idx]])

                # move the point from it's original position
                new_position = original_position + movement
                s._offsets[i] = new_position

                # adjust x values of all other points at this time as well
                for si in self.scatters:
                    si._offsets[i][0] = new_position[0]

                # move lines accordingly
                xdata = self.lines[joint_idx].get_xdata()
                ydata = self.lines[joint_idx].get_ydata()
                xdata[i] = new_position[0]
                ydata[i] = new_position[1]
                self.lines[joint_idx].set_xdata(xdata)
                self.lines[joint_idx].set_ydata(ydata)
                
                # redraw figure
                self.draw()


        # check if a point was hovered
        if self.selection_rect is None:
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
                        self.hovered[0].get_sizes()[self.hovered[1]] = self.default_size

                    # mark as hovered and increase size of this point
                    self.hovered = (s, index)
                    s.get_sizes()[index] = self.default_size * 2

                    # redraw canvas
                    self.draw()

                    return
        
        # if no point was touched, deselect and unhover
        if not self.hovered is None:
            self.hovered[0].get_sizes()[self.hovered[1]] = self.default_size
            self.hovered = None
            self.draw()

        # if no point was touched and the mouse is grabbing, draw selection rect
        if self.hovered is None and not self.grabbed is None:

            if not self.selection_rect is None:
                self.selection_rect.remove()

            difference = np.array([event.xdata, event.ydata]) - self.grabbed

            self.selection_rect = Rectangle(self.grabbed, difference[0], difference[1],
                                            linewidth=1, edgecolor='b', facecolor=(0.0, 0.2, 0.8, 0.2))
            self.axes.add_patch(self.selection_rect)

            # update selected points
            self.selected = []
            x1 = min(self.grabbed[0], event.xdata)
            x2 = max(self.grabbed[0], event.xdata)
            y1 = min(self.grabbed[1], event.ydata)
            y2 = max(self.grabbed[1], event.ydata)

            for scat in self.scatters:
                for i in range(len(scat._offsets)):
                    point = scat._offsets[i]

                    if x1 < point[0] and point[0] < x2 and y1 < point[1] and point[1] < y2:
                        self.selected.append((scat, i))

            self._update_selection()

    def _on_key_press(self, event):
        """
        Called when a key is pressed.
        """

        # update key down variables
        if event.key == 'control':
            self.key_ctrl = True
    
    def _on_key_release(self, event):
        """
        Called when a key is released.
        """

        # update key down variables
        if event.key == 'control':
            self.key_ctrl = False
    
    def on_enter_event(self, _):
        self.setFocus()

    # ---------------- SELECTION HELPERS ----------------------
    def _update_selection(self):
        """
        Set colors of all points according to selection
        """
        for s in self.scatters:
            for i in range(len(s.get_sizes())):

                # check if this point is selected
                if (s, i) in self.selected:
                    s.get_edgecolor()[i] = np.array([0.7, 0.7, 0.05, 1.0])
                
                # if not, it gets the default color
                else:
                    s.get_edgecolor()[i] = self.default_color
        
        # finally, redraw!
        self.draw()
