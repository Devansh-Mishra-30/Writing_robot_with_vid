import matplotlib.pyplot as plt
import numpy as np
import time
import imageio.v2 as imageio

class PandaPlot():
    def __init__(self, num_joints=7, max_points=500):
        self.NUM_JOINTS = num_joints
        self.DIM = 3
        self.max_points = max_points
        plt.ion()

        # Joint Position Plot
        self.fig_joints, self.fig_axes = plt.subplots(self.NUM_JOINTS, 1, figsize=(8, 12))
        self.lines_q = [ax.plot([], [], color="green")[0] for ax in self.fig_axes]
        self.lines_q_des = [ax.plot([], [], color="orange", linestyle="--")[0] for ax in self.fig_axes]
        for ax in self.fig_axes:
            ax.set_xlim(0, 60)
            ax.set_ylim(-3.14, 3.14)
            ax.grid(True)

        self.t_history = []
        self.q_history = [[] for _ in range(self.NUM_JOINTS)]
        self.q_des_history = [[] for _ in range(self.NUM_JOINTS)]

        # Plane Plots
        self.fig_planes, (self.ax_yx, self.ax_yz) = plt.subplots(1, 2, figsize=(12, 6))
        self.line_yx, = self.ax_yx.plot([], [], label="XY", color="green")
        self.line_yx_des, = self.ax_yx.plot([], [], label="XY Desired", color="orange", linestyle="--")
        self.line_yz, = self.ax_yz.plot([], [], label="YZ", color="green")
        self.line_yz_des, = self.ax_yz.plot([], [], label="YZ Desired", color="orange", linestyle="--")

        # XY Plane Setup
        self.ax_yx.set_title("XY Plane")
        self.ax_yx.set_xlabel("Y [m]")
        self.ax_yx.set_ylabel("X [m]")
        self.ax_yx.set_xlim(-0.25, 0.25)
        self.ax_yx.set_ylim(0.1, 0.6)
        self.ax_yx.invert_xaxis()
        self.ax_yx.grid(True)

        # YZ Plane Setup
        self.ax_yz.set_title("YZ Plane")
        self.ax_yz.set_xlabel("Y [m]")
        self.ax_yz.set_ylabel("Z [m]")
        self.ax_yz.set_xlim(-0.25, 0.25)
        self.ax_yz.set_ylim(0.63, 0.69)
        self.ax_yz.grid(True)

        self.Y_history = [[] for _ in range(self.DIM)]
        self.Y_des_history = [[] for _ in range(self.DIM)]

        # Writing Plot (Pen down only)
        self.fig_write, self.ax_write = plt.subplots()
        self.ax_write.set_title("XY Writing Path (Pen Down Only)")
        self.ax_write.set_xlabel("Y [m]")
        self.ax_write.set_ylabel("X [m]")
        self.ax_write.set_xlim(-0.25, 0.25)
        self.ax_write.set_ylim(0.1, 0.60)
        self.ax_write.invert_xaxis()
        self.ax_write.grid(True)
        self.write_x = []
        self.write_y = []

        # Frame storage
        self.frames_xy = []
        self.frames_yz = []
        self.frames_write = []

    def update_plot(self, t, q, q_des, Y, Y_des):
        # Update joint data
        self.t_history.append(t)
        for i in range(self.NUM_JOINTS):
            self.q_history[i].append(q[i])
            self.q_des_history[i].append(q_des[i])

        # Update joint plot lines
        for i, ax in enumerate(self.fig_axes):
            self.lines_q[i].set_data(self.t_history, self.q_history[i])
            self.lines_q_des[i].set_data(self.t_history, self.q_des_history[i])

        # Update plane data
        for i in range(self.DIM):
            self.Y_history[i].append(Y[i])
            self.Y_des_history[i].append(Y_des[i])

        # Update XY and YZ Planes
        self.line_yx.set_data(self.Y_history[1], self.Y_history[0])  # Y vs X
        self.line_yx_des.set_data(self.Y_des_history[1], self.Y_des_history[0])
        self.line_yz.set_data(self.Y_history[1], self.Y_history[2])  # Y vs Z
        self.line_yz_des.set_data(self.Y_des_history[1], self.Y_des_history[2])

        # Clean writing logic using Y_des[2] height marker
        if abs(Y_des[2] - 0.685) < 1e-5:
            # Pen lifted - start a new stroke
            self.write_x = []
            self.write_y = []
        else:
            self.write_x.append(Y[1])  # Y = horizontal
            self.write_y.append(Y[0])  # X = vertical
            if len(self.write_x) > 1:
                self.ax_write.plot(self.write_x[-2:], self.write_y[-2:], color="black", linewidth=1)

        plt.pause(0.001)

        # Capture video frames
        for fig, buffer in zip([self.fig_planes, self.fig_write], [self.frames_xy, self.frames_write]):
            fig.canvas.draw()
            image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            buffer.append(image)

        # Capture YZ frame
        self.fig_planes.canvas.draw()
        image_yz = np.frombuffer(self.fig_planes.canvas.tostring_rgb(), dtype='uint8')
        image_yz = image_yz.reshape(self.fig_planes.canvas.get_width_height()[::-1] + (3,))
        self.frames_yz.append(image_yz)

    def save_videos(self):
        with imageio.get_writer("XY_1.mp4", fps=10) as writer:
            for frame in self.frames_xy:
                writer.append_data(frame)

        with imageio.get_writer("YZ_1.mp4", fps=10) as writer:
            for frame in self.frames_yz:
                writer.append_data(frame)

        with imageio.get_writer("writing_video_1.mp4", fps=10) as writer:
            for frame in self.frames_write:
                writer.append_data(frame)