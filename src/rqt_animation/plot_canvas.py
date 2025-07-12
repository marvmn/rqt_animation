# Matplotlib
import matplotlib
import matplotlib.patheffects
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle, Polygon
from matplotlib.backend_tools import Cursors
from matplotlib.backend_bases import MouseButton

# QT
from python_qt_binding.QtGui import QCursor
from python_qt_binding.QtCore import Qt

# Python
import numpy as np
import copy

# custom stuff
from expressive_motion_generation.animation import BezierCurve
from expressive_motion_generation.trajectory import Trajectory
from expressive_motion_generation.effects import BezierCurveEffect

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, update_callback, background, parent=None, width=10, height=4, dpi=100):

        # initialize figure
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.fig.tight_layout(pad=0.0)
        self.fig.set_facecolor(background)
        self.axes.set_facecolor(background)
        self.update_callback = update_callback
        self.parent = parent

        # is the figure in advanced bezier mode?
        self.bezier_mode = False

        # bezier curves and their layers in the advanced view
        self.beziers = []
        self.bezier_layers = []

        # joint position limits
        self.joint_limits = []
        self.joint_limit_lines = []

        # currently shown bezier interval and corresponding bezier
        self.current_interval_index = -1
        self.current_bezier_index = -1

        # figure elements
        self.positions = []
        self.times = []
        self.bezier_blocks = []
        self.control_points = []
        self.control_lines = []
        self.check_mark = []

        # block where a bezier can be drawn in
        self.drawing_space = None
        self.drawn_bezier = None

        # rectangle for showing interval
        self.interval_rect = None

        # input event handlers
        self.handlers = []

        # default values
        self.default_color = np.array([0.1, 0.2, 0.6, 1.0])
        self.default_size = 64

        # current colors
        self.current_scatter_color = self.default_color

        # list of vertical lines that mark the positions of keyframes
        self.timebars = None

        # vertical line that indicates which position is currently selected
        self.indicator = None

        # currently selected points
        self.selected = []
        self.hovered = None

        # if the mouse has been held down, save the point where it was first pressed
        self.grabbed = None

        # rectangle that appears when dragging the mouse to select something or to zoom
        self.selection_rect = None
        self.zoom_rect = None

        # are these keys pressed currently?
        self.key_ctrl = False

        super().__init__(self.fig)
    
    def load_animation(self, positions, times, beziers, rescale=True):
        '''
        Plot animation file
        '''
        # save information
        self.positions = positions
        self.times = times
        self.beziers = beziers

        old_xlim = self.axes.get_xlim()
        old_ylim = self.axes.get_ylim()

        # create trajectory with applied bezier curves
        trajectory = Trajectory(times, positions)
        original_indices = trajectory.fill_up(10)
        for bezier in beziers:
            BezierCurveEffect(bezier.indices[0],
                                bezier.indices[1],
                                bezier.control_point0, bezier.control_point1).apply(trajectory)

        # delete old plot if there is one
        self.fig.clear(False)
        self.axes = self.fig.add_subplot(111)
        self.axes.set_facecolor(self.fig.get_facecolor())

        # draw animation lines and points
        self.lines = []
        self.scatters = []
        self.line_colors = []

        sizes = np.ones(len(times)) * self.default_size
        colors = np.tile(self.current_scatter_color, (len(times), 1))
        

        for i in range(len(positions.T)):

            # draw line
            l, = self.axes.plot(trajectory.times, trajectory.positions.T[i], linewidth=1, zorder=2)
            self.line_colors.append(l.get_color())
            self.lines.append(l)

            # draw point (and make it pickable)
            s = self.axes.scatter(times, positions.T[i], marker=".", s=copy.deepcopy(sizes), c=copy.deepcopy(colors), zorder=3)
            s.set_picker(True)
            s.set_label("joint" + str(i))
            self.scatters.append(s)

            # if bezier mode is active, make colors a bright grey
            if self.bezier_mode:
                line_color = np.array([1, 1, 1]) / (np.array([1, 1, 1]) +
                                                    np.array([matplotlib.colors.to_rgb(l.get_color())])
                                                    / 2)
                l.set_color(line_color[0])
                s.set_color(self.default_color * np.array([1.2, 1.2, 1.2, 1.0]))
        
        # adjust plot size
        if rescale:
            self.fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.15)
            self.adjust_yrange()
        else:
            self.axes.set_xlim(old_xlim)
            self.axes.set_ylim(old_ylim)

        # connect events
        self.handlers.append(self.mpl_connect('button_press_event', self._on_mouse_press))
        self.handlers.append(self.mpl_connect('button_release_event', self._on_mouse_release))
        self.handlers.append(self.mpl_connect('motion_notify_event', self._on_mouse_move))
        self.handlers.append(self.mpl_connect('key_press_event', self._on_key_press))
        self.handlers.append(self.mpl_connect('key_release_event', self._on_key_release))
        self.handlers.append(self.mpl_connect('axes_enter_event', self._on_enter_event))
        self.handlers.append(self.mpl_connect('figure_leave_event', self._on_leave_event))
    
    def load_advanced_beziers(self):
        """
        Loads all Bezier curves as blocks onto the canvas
        """

        # go through all beziers and create blocks for them
        # first sort them into layers so that there are no overlapping blocks
        self.bezier_layers = []

        for bezier in self.beziers:
            
            sorted = False

            for layer in self.bezier_layers:

                # if layer is empty OR if no block in the layer overlaps with this one, put it in this layer
                if not layer or \
                   not any(bezier.indices[0] <= block.indices[0] and bezier.indices[1] >= block.indices[0] or
                             bezier.indices[0] <= block.indices[1] and bezier.indices[1] >= block.indices[1] or
                             bezier.indices[0] >= block.indices[0] and bezier.indices[1] <= block.indices[1]
                             for block in layer):
                    layer.append(bezier)
                    sorted = True
                    break
                
                # otherwise, try the next layer
            
            # if no layer was found for this bezier, create new layer and put this one in
            if not sorted:
                self.bezier_layers.append([bezier])
        
        # now draw boxes for every layer, leave one layer for new boxes
        height = (self.axes.get_ylim()[1] - self.axes.get_ylim()[0]) * 0.8
        bottom = sum(self.axes.get_ylim())/2 - height/2
        box_height = height / (len(self.bezier_layers) + 1)

        for layer_id in range(len(self.bezier_layers)):
            for bezier_id in range(len(self.bezier_layers[layer_id])):
                bezier = self.bezier_layers[layer_id][bezier_id]
                rect = Rectangle([self.times[bezier.indices[0]], bottom + layer_id * box_height],
                                 self.times[bezier.indices[1]] - self.times[bezier.indices[0]],
                                 box_height * 0.8, linewidth=1, zorder=4, facecolor=[np.random.random((1))[0], 0.2, 0.5, 0.8],
                                 label=f'{layer_id}:{bezier_id}')
                self.axes.add_patch(rect)
                self.bezier_blocks.append(rect)
        
        # mark drawing space for new boxes
        self.drawing_space = Rectangle([self.times[0], bottom + len(self.bezier_layers) * box_height],
                                       self.times[-1] - self.times[0], box_height, zorder=3, facecolor=[0.8, 0.6, 0.0, 0.2],
                                       edgecolor='yellow', linewidth=1, linestyle='--')
        self.axes.add_patch(self.drawing_space)

        self.draw_idle()

    def unload_advanced_beziers(self):
        """
        Unloads and deletes all advanced bezier elements
        """

        # remove all ezier blocks
        for i in range(len(self.bezier_blocks)):
            block = self.bezier_blocks.pop()
            block.remove()
        
        # clear layer list
        self.bezier_layers = []

        # remove drawing space
        if self.drawing_space is not None:
            self.drawing_space.remove()
            self.drawing_space = None
        
        # remove checkmark
        if self.check_mark:
            self._remove_checkmark()
        
        # reset hovered in case the mouse was above an element before unloading
        self.hovered = None

    def get_bounds(self):
        """
        Returns the x pixel coordinates of the left and right bounds of the plot
        """
        left_x = self.axes.transData.transform([self.times[0], 0])[0]
        right_x = self.axes.transData.transform([self.times[-1], 0])[0]
        return left_x, right_x

    def set_xrange(self, low, high):
        self.axes.set_xlim(low, high)
        self.draw_idle()
    
    def adjust_yrange(self):
        """
        Sets y range to fit the data
        """
        max = np.max(self.positions)
        min = np.min(self.positions)
        padding = abs(max) - abs(min) * 0.1
        self.axes.set_ylim(min - padding, max + padding)

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
        self.draw_idle()
    
    def unload(self):
        """
        Removes all plot data
        """
        self.times = None
        self.positions = None
        self.beziers = None

        self.lines = []
        self.scatters = []

        for id in self.handlers:
            self.mpl_disconnect(id)

        self.fig.clear(False)
        self.draw()

    def toggle_bezier_mode(self):
        """
        Toggles the view between normal keyframe view and bezier curve edit view
        """
        self.bezier_mode = not self.bezier_mode

    def remove_selection(self):
        '''
        Deletes all selected keyframes
        '''
        if self.bezier_mode:
            # check if a bezier curve is selected
            if self.current_bezier_index >= 0:
                # delete bezier
                self.beziers.pop(self.current_bezier_index)
                self.update_callback()

                # go back to block view
                self._remove_bezier_selection()
                self.load_advanced_beziers()
        else:
            # check which keyframe indices need to be removed
            deleted = []
            for (s, i) in self.selected:
                if not i in deleted:
                    deleted.append(i)
            
            # remove them from latest to earliest  
            deleted.sort(reverse=True)
            for i in deleted:
                self.times = np.delete(self.times, i)
                self.positions = np.delete(self.positions, i, axis=0)

                # update bezier indices
                for bezier in self.beziers:
                    if bezier.indices[0] >= i:
                        bezier.indices = (bezier.indices[0] - 1, bezier.indices[1] - 1)
                    elif bezier.indices[1] >= i:
                        bezier.indices = (bezier.indices[0], bezier.indices[1] - 1)
                    
                    # if bezier does not contain any frames anymore, delete it
                    if bezier.indices[0] >= bezier.indices[1]:
                        #self.beziers.remove(bezier)
                        pass

    def set_joint_limits(self, limits):
        '''
        Set the joint position limits
        '''
        self.joint_limits = limits

    def get_override_joint_state(self):
        """
        If the current joint state is being edited, returns the
        correct current joint state
        """
        # check if something is being grabbed
        if self.selected and not self.grabbed is None and self.selection_rect is None and self.zoom_rect is None:

            # prepare position list
            time_index = self.hovered[1]
            position_list = copy.deepcopy(self.positions[time_index])

            # search for changed joints
            for (s, i) in self.selected:

                # if this was a joint position point, update joint position
                if s in self.scatters:
                    joint_idx = int(s.get_label()[len('joint'):])
                    position_list[joint_idx] = s._offsets[i][1]
            
            # return list
            return position_list
        
        # if nothing is grabbed, return nothing
        return None

    def get_highlight(self):
        """
        ! NOT IMPLEMENTED
        If a joint is being hovered, return which joint index it is so that
        the corresponding link can be highlighted in the robot state display.
        """

        # first, check if a joint is being hovered
        if type(self.hovered) == tuple:
            s, _ = self.hovered
            if s in self.scatters:

                # get which link this joint belongs to
                joint_idx = int(s.get_label()[len('joint'):])
                return joint_idx
    
        # if nothing was found return None
        return None

    # ------------------- EVENT HANDLERS ---------------------

    def _on_mouse_press(self, event):
        """
        Called when the user presses a mouse button in the plot window
        """

        # save the point where the mouse button was clicked
        self.grabbed = [event.xdata, event.ydata]
        
        # check if the mouse was hovering over a point and clicked it now
        if not self.hovered is None and event.button == MouseButton.LEFT:
            
            # if self.hovered is a tuple, a point is selected
            if type(self.hovered) == tuple:
                # select point
                self.selected.append(self.hovered)

                # make the point a little bit bigger
                self.hovered[0].get_sizes()[self.hovered[1]] = self.default_size * 5

                # show selection
                self._update_selection()
            
            # otherwise, a bezier block or the drawing space is selected
            elif self.hovered == self.drawing_space:
                # start drawing
                pass
            else:
                # remove the grabbed flag, because this is a button press
                self.grabbed = None

                # select this bezier curve and open editor
                indices = self.hovered.get_label().split(':')
                bezier = self.bezier_layers[int(indices[0])][int(indices[1])]

                self.current_bezier_index = self.beziers.index(bezier)
                self.current_interval_index = bezier.indices[0]
                self.unload_advanced_beziers()

                # draw curve and control points
                interval_size = bezier.indices[1]-bezier.indices[0]
                self._draw_bezier_interval(interval_size)
        
        # if not, nothing was clicked on, so we deselect everything
        else:
            self.selected = []
            self._update_selection()

            # also, if advanced bezier mode is active and the mouse is outside of the interval,
            # clicking should go back to the block view
            if self.control_points and event.xdata < self.times[self.current_interval_index] \
                                    or event.xdata > self.times[self.beziers[self.current_bezier_index].indices[1]]:
                
                self._remove_bezier_selection()
                self._remove_checkmark()
                self.load_advanced_beziers()
                self.draw_timebars(None)
            
    def _on_mouse_release(self, event):
        """
        Called when the user releases a mouse button
        """

        # if there is a selection and no rect, update times and positions values according to
        # plot values (if they were moved while the mouse was held down)
        if self.selected and self.selection_rect is None and self.zoom_rect is None:

            for (s, i) in self.selected:

                # if this was a joint position point, update joint positions and times
                if s in self.scatters:
                    joint_idx = int(s.get_label()[len('joint'):])

                    self.times[i] = s._offsets[i][0]
                    self.positions[i][joint_idx] = s._offsets[i][1]
                
                # if this was a bezier curve control point, update bezier curve
                elif s in self.control_points:
                    new_position = (np.array(s._offsets[0]) \
                                    - np.array([self.times[self.current_interval_index], -self.interval_diff[1]/2])) / self.interval_diff
                    if s == self.control_points[0]:
                        self.beziers[self.current_bezier_index].control_point0 = new_position
                    else:
                        self.beziers[self.current_bezier_index].control_point1 = new_position
                    
                    # reset values if this is not advanced editing mode
                    if not self.bezier_mode:
                        self._remove_bezier_selection()

            if self.bezier_mode:
                self.selected = []
                self._update_selection()
            else:
                # redraw time bars
                self.draw_timebars(None)

                # remove selection rect
                # self.selection_rect = None
                self.selected = []

                # update animation data in parent widget
                self.update_callback()
        
        # if advanced bezier editing mode is active, don't do anything from this point onwards except
        # drawn bezier block handling
        if self.bezier_mode:
            self.grabbed = None

            # if there is a drawn bezier, save it as new curve and go into editor!
            if self.drawn_bezier is not None:
                first_index = np.abs(self.times - self.drawn_bezier.get_x()).argmin()
                second_index = np.abs(self.times - self.drawn_bezier.get_x() - self.drawn_bezier.get_width()).argmin()
                self.drawn_bezier.remove()
                self.drawn_bezier = None

                # if the indices are the same, don't create a new bezier and cancel
                if not first_index == second_index:
                    bezier = BezierCurve((first_index, second_index),
                                         np.array([0.2, 0.2]), np.array([0.8, 0.8]))
                    self.beziers.append(bezier)

                    self.current_bezier_index = len(self.beziers) - 1
                    self.current_interval_index = bezier.indices[0]
                    self.unload_advanced_beziers()

                    # draw curve and control points
                    interval_size = bezier.indices[1]-bezier.indices[0]
                    self._draw_bezier_interval(interval_size)

            return

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
            self.draw_idle()
        
        # zoom if a zoom rect was drawn
        if not self.zoom_rect is None:

            x = self.zoom_rect.get_x()
            y = self.zoom_rect.get_y()
            w = self.zoom_rect.get_width()
            h = self.zoom_rect.get_height()

            # remove rect
            self.zoom_rect.remove()
            self.zoom_rect = None

            # set axe limits
            self.axes.set_xlim(x, x+w)
            self.axes.set_ylim(y, y+h)

            # redraw
            self.draw_idle()

    def _on_mouse_move(self, event):
        """
        Called when the user moves their mouse
        """
        # check if mouse went out of bounds, if so stop this
        if event.xdata is None or event.ydata is None:
            return

        # update currently hovered bezier interval if not in advanced mode
        if not self.bezier_mode:
            # if mouse is out of frame or there is a selection,
            # and if there is still a rect, remove it
            if (event.xdata is None or event.ydata is None and not self.interval_rect is None) \
                or not self.selection_rect is None or not self.zoom_rect is None or (self.selected and self.selected[0][0] in self.scatters):

                self._remove_bezier_selection()
            
            # otherwise, if a point is not being dragged currently, change the bezier interval
            elif not self.selected:

                # find interval index
                index = -1
                for i in range(len(self.times)):
                    if event.xdata < self.times[i]:
                        index = i - 1
                        break
                
                # if an interval was found and it's not the same as before, select it
                if not index < 0 and not self.current_interval_index == index:
                    self._remove_bezier_selection()
                    self.current_interval_index = index

                    # find correct bezier curve
                    for i in range(len(self.beziers)):
                        if self.beziers[i].indices[0] == self.current_interval_index and self.beziers[i].indices[1] == self.current_interval_index + 1:
                            self.current_bezier_index = i
                            break
                    
                    # if no curve was found, create new
                    if self.current_bezier_index == -1:
                        bezier = BezierCurve(indices=(self.current_interval_index, self.current_interval_index + 1),
                                            control_point0=np.array([0.2, 0.2]), control_point1=np.array([0.8, 0.8]))
                        self.current_bezier_index = len(self.beziers)
                        self.beziers.append(bezier)

                    self._draw_bezier_interval()
                
                elif index < 0:
                    self._remove_bezier_selection()

        # if in advanced bezier mode, check if the mouse is outside the interval to
        # display the checkmark (only when a curve is loaded)
        else:
            if self.control_points and (event.xdata < self.times[self.current_interval_index] \
                                   or event.xdata > self.times[self.beziers[self.current_bezier_index].indices[1]]):
                self._draw_checkmark_at(event.xdata, event.ydata)
            else:
                self._remove_checkmark()
            
            # if a block is drawn, display it
            if self.hovered is not None and self.hovered == self.drawing_space and self.grabbed:
                self._draw_new_bezier_block(event.xdata)

        # check if something is selected and the mouse button has been pressed
        # while there is no selection rect.
        # That means that the current selection is being moved.
        if self.selected and not self.grabbed is None and self.selection_rect is None and self.zoom_rect is None:
            # something is selected and the mouse was dragged
            # move selected points the specified amount
            movement = np.array([event.xdata, event.ydata]) - self.grabbed

            # if CTRL was held down, only move on the y axis
            if self.key_ctrl:
                movement[0] = 0.0

            for (s, i) in self.selected:

                # if this was a joint point, move it and adjust joint positions and times
                if s in self.scatters:
                    joint_idx = int(s.get_label()[len('joint'):])
                    original_position = np.array([self.times[i], self.positions[i][joint_idx]])

                    # move the point from it's original position
                    new_position = original_position + movement

                    # check if joint constraints are being violated
                    new_position[1] = max(new_position[1], self.joint_limits[joint_idx][0])
                    new_position[1] = min(new_position[1], self.joint_limits[joint_idx][1])

                    s._offsets[i] = new_position

                    # adjust x values of all other points at this time as well
                    for si in self.scatters:
                        si._offsets[i][0] = new_position[0]

                    # replot lines accordingly
                    trajectory = Trajectory(copy.deepcopy(self.times), copy.deepcopy(self.positions))
                    trajectory.times[i] = new_position[0]
                    trajectory.positions[i][joint_idx] = new_position[1]
                    original_indices = trajectory.fill_up(10)
                    for bezier in self.beziers:
                        BezierCurveEffect(bezier.indices[0],
                                        bezier.indices[1],
                                        bezier.control_point0, bezier.control_point1).apply(trajectory)

                    for _ in range(len(self.lines)):
                        line = self.lines.pop()
                        line.remove()
                    
                    for i in range(len(self.positions.T)):
                        # draw line
                        l, = self.axes.plot(trajectory.times, trajectory.positions.T[i], linewidth=1, zorder=2, c=self.line_colors[i])
                        self.lines.append(l)
                
                # if it was a bezier control point, adjust bezier parameters
                elif s in self.control_points:

                    # limit the mouse to the current interval
                    interval_length = self.beziers[self.current_bezier_index].indices[1] - self.beziers[self.current_bezier_index].indices[0]
                    event.xdata = np.max([event.xdata, self.times[self.current_interval_index]])
                    event.xdata = np.min([event.xdata, self.times[self.current_interval_index + interval_length]])
                    movement = np.array([event.xdata, event.ydata]) - self.grabbed

                    if s == self.control_points[0]:
                        original_position = np.array(self.beziers[self.current_bezier_index].control_point0) * self.interval_diff \
                                            + np.array([self.times[self.current_interval_index], -self.interval_diff[1]/2])
                        c = 0
                    
                    else:
                        original_position = np.array(self.beziers[self.current_bezier_index].control_point1) * self.interval_diff \
                                            + np.array([self.times[self.current_interval_index], -self.interval_diff[1]/2])
                        c = 1
                    
                    # calculate new bezier point
                    new_position = (original_position + movement \
                                    - np.array([self.times[self.current_interval_index], -self.interval_diff[1]/2])) / self.interval_diff

                    # adjust point
                    s._offsets[0] = original_position + movement

                    # adjust control line
                    xdata = self.control_lines[c][0].get_xdata()
                    ydata = self.control_lines[c][0].get_ydata()
                    xdata[1] = s._offsets[0][0]
                    ydata[1] = s._offsets[0][1]
                    self.control_lines[c][0].set_xdata(xdata)
                    self.control_lines[c][0].set_ydata(ydata)

                    # redraw bezier curve
                    interval_length = self.beziers[self.current_bezier_index].indices[1] - self.beziers[self.current_bezier_index].indices[0]
                    if c == 0:
                        self._draw_faint_bezier_curve(new_position, self.beziers[self.current_bezier_index].control_point1, interval_length)
                    else:
                        self._draw_faint_bezier_curve(self.beziers[self.current_bezier_index].control_point0, new_position, interval_length)
                
                
            # redraw figure
            self.draw_idle()
            return


        # check if a point was hovered
        if self.selection_rect is None and self.zoom_rect is None:

            if not self.bezier_mode:
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

                        # draw joint limit lines
                        self._draw_joint_limit_lines(int(s.get_label()[len('joint'):]))

                        # redraw canvas
                        self.set_cursor(Cursors.HAND)
                        self.draw_idle()

                        return
                
            else:
                
                if self.drawing_space is not None and self.drawing_space.get_bbox().contains(event.xdata, event.ydata):

                    self.set_cursor(Cursors.SELECT_REGION)
                    self.hovered = self.drawing_space
                    return

                for block in self.bezier_blocks:
                    if block.get_bbox().contains(event.xdata, event.ydata):

                        self.set_cursor(Cursors.HAND)
                        
                        # check if another block has been hovered before
                        if not block == self.hovered and not self.hovered is None and not self.hovered == self.drawing_space:
                            self.hovered.set(linewidth=0)

                        block.set(edgecolor='yellow', linewidth=1)
                        self.hovered = block
                        self.draw_idle()
                        return

            # always check for control points, since they can exist in both modes
            for s in self.control_points:
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
                    self.set_cursor(Cursors.HAND)
                    self.draw_idle()

                    return


        # if no point was touched, deselect and unhover
        if not self.hovered is None:

            # if a point is selected, self.hovered will be a tuple
            # in that case set previously hovered point back to normal size and remove limit lines
            if type(self.hovered) == tuple:
                self.hovered[0].get_sizes()[self.hovered[1]] = self.default_size
                self._draw_joint_limit_lines(None)
            
            # if not, it is a bezier block or the drawing space
            elif self.hovered == self.drawing_space:
                pass
            else:
                self.hovered.set(linewidth=0)

            self.hovered = None
            self.draw_idle()
            self.set_cursor(Cursors.POINTER)

        # if no point was touched and the mouse is grabbing, draw selection rect or zoom rect
        if self.hovered is None and not self.grabbed is None and not self.bezier_mode:
            if event.button == MouseButton.LEFT:
                rect = self.selection_rect
                color = 'b'
            else:
                rect = self.zoom_rect
                color = 'yellow'

            if not rect is None:
                # check if mouse went out of bounds and keep old values if so
                if event.xdata is None:
                    event.xdata = rect.get_width() + self.grabbed[0]
                if event.ydata is None:
                    event.ydata = rect.get_height() + self.grabbed[1]
                
                # remove old rect
                rect.remove()
            
            # calculate size of rect
            difference = np.array([event.xdata, event.ydata]) - self.grabbed

            rect = Rectangle(self.grabbed, difference[0], difference[1],
                             linewidth=1, edgecolor=color, facecolor=(0.0, 0.2, 0.8, 0.2), zorder=1)
            self.axes.add_patch(rect)

            if event.button == MouseButton.LEFT:
                # set selection rect to rect
                self.selection_rect = rect

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

            else:
                self.zoom_rect = rect
                self.draw_idle()

    def _on_key_press(self, event):
        """
        Called when a key is pressed.
        """

        # update key down variables
        if event.key == 'control':
            self.key_ctrl = True
        if event.key == 'shift':
            self.key_shift = True
    
    def _on_key_release(self, event):
        """
        Called when a key is released.
        """
        # update key down variables
        if event.key == 'control':
            self.key_ctrl = False
        if event.key == 'shift':
            self.key_shift = False
        
        # file shortcuts
        if self.key_ctrl:
            if event.key == 'ctrl+S':
                self.parent._on_saveAsButton_clicked()
            elif event.key == 'ctrl+s':
                self.parent._on_saveButton_clicked()
            elif event.key == 'ctrl+n':
                self.parent._on_newButton_clicked()
            elif event.key == 'ctrl+o':
                self.parent._on_openButton_clicked()
        
    def _on_enter_event(self, event):
        self.setFocus()

    def _on_leave_event(self, event):
        '''
        When the mouse leaves this widget, remove selections
        '''
        self.hovered = None
        self.grabbed = None

        if self.bezier_mode:
            self._remove_checkmark()
        else:
            self._remove_bezier_selection()
        
        self.draw_idle()

    # ---------------- HELPERS ----------------------
    def _update_selection(self):
        """
        Set colors of all points according to selection
        """
        for s in self.scatters:
            for i in range(len(s.get_edgecolor())):

                # check if this point is selected
                if (s, i) in self.selected:
                    s.get_edgecolor()[i] = np.array([0.7, 0.7, 0.05, 1.0])
                
                # if not, it gets the default color
                else:
                    s.get_edgecolor()[i] = self.default_color
        
        for p in self.control_points:

            # check if this point is selected
            if (p, 0) in self.selected:
                p.get_edgecolor()[0] = np.array([0.3, 0.3, 0.9, 0.8])
            
            # if not, it returns to the grey color
            else:
                p.get_edgecolor()[0] = np.array([0.5, 0.5, 0.5, 0.5])
        
        # finally, redraw!
        self.draw_idle()

    def _draw_bezier_interval(self, interval_length=1):
        """
        Mark the background of the selected interval yellow, draw control points
        """
        # draw rect in background
        width = self.axes.get_xlim()[1] - self.times[self.current_interval_index] \
            if self.current_interval_index == len(self.times) - 1 else \
                self.times[self.current_interval_index + interval_length] - self.times[self.current_interval_index]

        self.interval_rect = Rectangle([self.times[self.current_interval_index], self.axes.get_ylim()[0] + 0.7], width, 
                                    abs(self.axes.get_ylim()[0]) + self.axes.get_ylim()[1] - 1.4,
                                    linewidth=0, facecolor=(0.8, 0.8, 0.0, 0.2), zorder=0)
        self.axes.add_patch(self.interval_rect)

        # load bezier and draw control points
        
        # create points
        color = np.array([0.5, 0.5, 0.5, 0.5])
        cii = self.current_interval_index
        cbi = self.current_bezier_index
        self.interval_diff = np.array([self.times[cii + interval_length] - self.times[cii],
                                        abs(self.axes.get_ylim()[0]) + self.axes.get_ylim()[1] - 1.4 ])
        
        point0 = [self.times[cii] + self.beziers[cbi].control_point0[0] * self.interval_diff[0],
                  -self.interval_diff[1]/2 + self.beziers[cbi].control_point0[1] * self.interval_diff[1]]
        point1 = [self.times[cii] + self.beziers[cbi].control_point1[0] * self.interval_diff[0],
                  -self.interval_diff[1]/2 + self.beziers[cbi].control_point1[1] * self.interval_diff[1]]
        
        scat0 = self.axes.scatter(point0[0], point0[1], c=color)
        scat1 = self.axes.scatter(point1[0], point1[1], c=color)
        
        line0 = self.axes.plot([self.times[cii], point0[0]],
                               [-self.interval_diff[1]/2, point0[1]], c=color)
        line1 = self.axes.plot([self.times[cii + interval_length], point1[0]],
                               [self.interval_diff[1]/2, point1[1]], c=color)

        self.control_points = [scat0, scat1]
        self.control_lines = [line0, line1]

        # draw faint curve line
        self._draw_faint_bezier_curve(self.beziers[cbi].control_point0, self.beziers[cbi].control_point1, interval_length)

        self.draw_idle()
    
    def _draw_faint_bezier_curve(self, control_point0, control_point1, interval_length=1):
        '''
        Draw a faint bezier curve in the interval that is specified by self.current_interval_index
        '''

        # define color and abbreviations for indices
        color = np.array([0.5, 0.5, 0.5, 0.5])
        cii = self.current_interval_index

        # create trajectory planner on the specified interval and apply bezier curve
        trajectory = Trajectory(np.linspace(self.times[cii], self.times[cii + interval_length], 20),
                                       np.linspace(-self.interval_diff[1]/2, self.interval_diff[1]/2, 20))
        BezierCurveEffect(0, 19, control_point0, control_point1).apply(trajectory)

        # plot
        line_bezier = self.axes.plot(trajectory.times, trajectory.positions, c=color)

        # if there was a line before, remove it
        if len(self.control_lines) == 3:
            l = self.control_lines.pop()
            l[0].remove()
        self.control_lines.append(line_bezier)

    def _remove_bezier_selection(self):
        """
        Removes bezier interval (rect and control points)
        """
        # if a control point is selected, remove selection
        if self.selected and any(point in self.control_points for (point, _) in self.selected):
            self.selected = []
        
        # if the background of the interval is marked yellow, remove marker
        if not self.interval_rect is None:
            self.interval_rect.remove()
            self.interval_rect = None

        # reset indices
        self.current_interval_index = -1
        self.current_bezier_index = -1

        # remove points and lines
        for scatter in self.control_points:
            scatter.remove()
        for line in self.control_lines:
            line[0].remove()
        self.control_points = []
        self.control_lines = []

    def _draw_checkmark_at(self, x, y):
        """
        Draws a green circle with a white check mark in it at the specified position
        """
        # save the axe limits to avoid rescaling through the checkmark position
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()

        # get axe ratio
        bbox = self.axes.get_window_extent().transformed(self.fig.dpi_scale_trans.inverted())
        ratio = bbox.width / bbox.height
        
        # remove old check mark if there is one
        if self.check_mark:
            for i in range(len(self.check_mark)):
                artist = self.check_mark.pop()
                artist.remove()
        
        # draw green circle
        circle = self.axes.scatter(x, y, 350, c='green', zorder=4)
        self.check_mark.append(circle)

        # draw check mark inside
        checkmark_shape = np.array([[-1.2 ,-1.0, -0.2,1.6,1.8, -0.2,-1.2],
                                    [0    , 0.2,- 0.3,1  ,0.8, -0.5, 0]]).T / (bbox.height * 15)
        mark = Polygon(checkmark_shape * np.array([0.4, ratio]) + np.array([x, y]), closed=True, zorder=5, color='white')
        self.axes.add_patch(mark)
        self.check_mark.append(mark)

        # apply old axe limits
        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)

        # redraw canvas
        self.set_cursor(Cursors.HAND)
        self.draw_idle()

    def _remove_checkmark(self):
        """
        Remove checkmark and set cursor back to default
        """
        if self.check_mark:
            for i in range(len(self.check_mark)):
                artist = self.check_mark.pop()
                artist.remove()
        self.set_cursor(Cursors.POINTER)
        self.draw_idle()

    def _draw_new_bezier_block(self, xdata):
        """
        Draw a new bezier block in the drawing space in advanced editing mode.
        Block starts at the mouse grabbed location and goes to the xdata coordinate.
        """

        # get indices for start and end
        end_coord = self.times[np.abs(self.times - xdata).argmin()]

        # delete old block if it exists
        if not self.drawn_bezier is None:
            start_coord = self.drawn_bezier.get_x()
            self.drawn_bezier.remove()
        else:
            start_coord = self.times[np.abs(self.times - self.grabbed[0]).argmin()]

        self.drawn_bezier = Rectangle([start_coord, self.drawing_space.get_y()],
                                      end_coord - start_coord, self.drawing_space.get_height(),
                                      facecolor='green')
        self.axes.add_patch(self.drawn_bezier)

    def _draw_joint_limit_lines(self, joint_index):
        """
        Displays two dashed lines for the lower and upper position
        limit for the specified joint.
        If joint_index is None, the old lines will just get deleted.
        """
        # remove old lines if there are any
        if self.joint_limit_lines:
            for _ in range(len(self.joint_limit_lines)):
                line = self.joint_limit_lines.pop()
                line.remove()
        
        # if joint index is None, exit here
        if joint_index is None:
            return

        # draw lower and upper line
        upper, = self.axes.plot([self.times[0], self.times[-1]],
                                [self.joint_limits[joint_index][0], self.joint_limits[joint_index][0]],
                                c='r', linestyle='-.', linewidth=0.75)
        
        lower, = self.axes.plot([self.times[0], self.times[-1]],
                                [self.joint_limits[joint_index][1], self.joint_limits[joint_index][1]],
                                c='r', linestyle='-.', linewidth=0.5)

        self.joint_limit_lines = [upper, lower]

    def set_cursor(self, cursor):
        """
        Set the mouse cursor to a predefined Cursor.
        This method is already implemented in some Qt Backends but not in
        all of them, so for compatibility it is reimplemented here.
        """        
        if cursor == Cursors.HAND:
            self.setCursor(QCursor(Qt.PointingHandCursor))
        elif cursor == Cursors.SELECT_REGION:
            self.setCursor(QCursor(Qt.CrossCursor))
        else:
            self.setCursor(QCursor(Qt.ArrowCursor))
