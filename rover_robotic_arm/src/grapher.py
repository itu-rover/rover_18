import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import math
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d

limits = 100


class Grapher():
    def __init__(self, lines):
        self.lines = lines
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.autoscale_view(False, False, False)

        self.ax.axis('equal')
        self.ax.set_xlim3d(-limits, limits)
        self.ax.set_ylim3d(-limits, limits)
        self.ax.set_zlim3d(-limits, limits)

        self.ax.autoscale(False)

        self.points = []
        self.base_lines = []

    def _circular(self, p, r):
        _p = Circle((p[1], p[2]), r)
        _p.set_facecolor("none")
        _p.set_edgecolor("blue")
        self.ax.add_patch(_p)
        art3d.pathpatch_2d_to_3d(_p, z=p[0], zdir="x")

    def __animate__(self, i):
        self.anim_f()
        self.ax.clear()
        for line in self.lines:
            self.ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], zs=[line[0][2], line[1][2]], linewidth=2.0, c='r')
            # self._circular([40,0,40], 10)
            self.ax.plot([-50., -50.1], [-50., -50.1], zs=[100, 100.1])
            self.ax.plot([50., 50.1], [50., 50.1], zs=[70, 70.1])

            # self.ax.plot(100,100,100)

        if len(self.base_lines) > 0:
            for line in self.base_lines:
                self.base_lines
                self.ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], zs=[line[0][2], line[1][2]], linewidth=0.7, c='b')

        if len(self.points) > 0:
            for p in self.points:
                self.ax.scatter(p[0], p[1], p[2], 'o', s=1)
                # pass

    def redraw(self, lines):
        self.lines = lines

    def redraw_base(self, lines):
        self.base_lines = lines

    def redraw_point(self, point):
        self.points.append(point)

    def clear_points(self):
        self.points = []

    def show(self, animate_function):
        self.anim_f = animate_function
        ani = animation.FuncAnimation(self.fig, self.__animate__, interval=100)
        plt.axis('equal')
        plt.autoscale(False)
        plt.show()
        Axes3D.plot()
