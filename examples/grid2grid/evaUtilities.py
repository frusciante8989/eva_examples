import matplotlib.pyplot as plt
from config.config_manager import load_use_case_config
import numpy as np
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch, FancyArrowPatch
import mpl_toolkits.mplot3d.art3d as art3d
import mpl_toolkits.mplot3d as mp3d
import matplotlib.patches as mpatches


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def wrap_to_pi(angle):
    # Wrap to 360
    ang_2pi = angle - (angle // (2 * np.pi)) * 2*np.pi
    if ang_2pi > np.pi or ang_2pi < - np.pi:
        # Wrap to 180
        ang_pi = ang_2pi-np.sign(ang_2pi)*2*np.pi
        return ang_pi


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


def solve_ik(_eva, guess, theta, xyz_absolute):
    with _eva.lock():
        # Inputs: Theta [deg], xyz_absolute [m]
        pos = [xyz_absolute[0], xyz_absolute[1], xyz_absolute[2]]  # [m]
        pos_json = {'x': (pos[0]), 'y': (pos[1]), 'z': (pos[2])}
        # This provides a rotation of theta [deg] wrt the end effector orientation pointing downwards
        orient_rel = [np.cos(np.deg2rad(theta)/2), 0, 0, np.sin(np.deg2rad(theta)/2)]
        orient_abs = quaternion_multiply([0, 0, 1, 0], orient_rel)
        orient_json = {'w': (orient_abs[0]), 'x': (orient_abs[1]), 'y': (orient_abs[2]), 'z': (orient_abs[3])}
        # Compute IK
        result_ik = _eva.calc_inverse_kinematics(guess, pos_json, orient_json)
        success_ik = result_ik['ik']['result']
        joints_ik = result_ik['ik']['joints']
    return success_ik, joints_ik


def solve_fk(eva, joints):
    with eva.lock():
        fk_results = eva.calc_forward_kinematics(joints)
        pos_json = fk_results['position']
        orient_json = fk_results['orientation']
        pos = [pos_json['x'], pos_json['y'], pos_json['z']]
        orient = [orient_json['w'], orient_json['x'], orient_json['y'], orient_json['z']]
    return pos, orient


def plot_grids(grid_x, grid_y, grid_z, ax_all, _grid_iter):
    all_points = []
    ax1 = ax_all[0]
    ax2 = ax_all[1]

    # Plot settings
    grid_color = {'A': 'blue', 'B': 'red'}
    box_side = 600
    arr_length = 300
    work = Circle((0, 0), config['EVA']['work_radius']*1000, color='k', alpha=0.1)
    base = config['EVA']['base_plate']*1000/2

    # 2D Plot
    base_plate_2d = FancyBboxPatch((-base, -base), base*2, base*2, edgecolor='black', facecolor='gray', alpha=0.2)
    io_port_2d = Rectangle((-0.08*1000, -0.08*1000/2), 0.015*1000, 0.08*1000, edgecolor='green', facecolor='green')
    grid_vertices_2d = [(grid_x[0][0], grid_y[0][0]), (grid_x[-1][0], grid_y[-1][0]),
                        (grid_x[-1][-1], grid_y[-1][-1]), (grid_x[0][-1], grid_y[0][-1])]
    grid_footprint_2d = mp3d.art3d.PolyCollection([grid_vertices_2d], color=grid_color[_grid_iter], alpha=0.3)
    ax1.add_artist(plt.Circle((0, 0), config['EVA']['work_radius'] * 1000, facecolor='gray', alpha=0.1))
    ax1.add_patch(base_plate_2d), ax1.add_patch(io_port_2d)
    ax1.set_xlim(-box_side, box_side), ax1.set_ylim(-box_side, box_side)
    ax1.set_aspect('equal'), ax1.set_xlabel('x [mm]'), ax1.set_ylabel('y [mm]')
    ax1.arrow(0, 0, 200, 0, head_width=10, head_length=10, fc='red', ec='red', linewidth=2, alpha=0.6)
    ax1.arrow(0, 0, 0, 200, head_width=10, head_length=10, fc='green', ec='green', linewidth=2, alpha=0.6)
    ax1.plot(0, 0, 'o', markersize=5, color='blue', alpha=0.6)
    ax1.plot(grid_x, grid_y, 'o', color='grey', markersize=2)
    ax1.plot(grid_x[0][0], grid_y[0][0], '*', markersize=10, color='green')  # First point
    ax1.plot(grid_x[-1][-1], grid_y[-1][-1], 'X', markersize=8, color='red')  # Last point
    ax1.add_collection(grid_footprint_2d)
    blue_patch = mpatches.Patch(color='blue', label='Origin grid', alpha=0.4)
    red_patch = mpatches.Patch(color='red', label='Target grid', alpha=0.4)
    plt.legend(handles=[blue_patch, red_patch])

    for i in range(config['grids']['row'][_grid_iter]):
        for j in range(config['grids']['col'][_grid_iter]):
            all_points.append([grid_x[i][j], grid_y[i][j]])

    for i in range(len(all_points)-1):
        ax1.arrow(all_points[i][0], all_points[i][1], (all_points[i+1][0]-all_points[i][0]), (all_points[i+1][1]-all_points[i][1]), head_width=15, head_length=15, fc='black', ec='black', linewidth=1, alpha=0.5)

    # 3D Plot
    base_plate_3d = [(-base, -base, 0), (base, -base, 0), (base, base, 0), (-base, base, 0)]
    base_plate_shape_3d = mp3d.art3d.Poly3DCollection([base_plate_3d], color='black', alpha=0.5, linewidth=1)
    grid_vertices_3d = [(grid_x[0][0], grid_y[0][0], grid_z[0][0]), (grid_x[-1][0], grid_y[-1][0], grid_z[0][0]),
                        (grid_x[-1][-1], grid_y[-1][-1], grid_z[0][0]),
                        (grid_x[0][-1], grid_y[0][-1], grid_z[0][0])]
    grid_footprint_3d = mp3d.art3d.Poly3DCollection([grid_vertices_3d], color=grid_color[_grid_iter], alpha=0.3)

    a = Arrow3D([0, arr_length], [0, 0], [0, 0], mutation_scale=5, lw=1, arrowstyle="-|>", color="r")
    b = Arrow3D([0, 0], [0, arr_length], [0, 0], mutation_scale=5, lw=1, arrowstyle="-|>", color="g")
    c = Arrow3D([0, 0], [0, 0], [0, 0.5*arr_length], mutation_scale=5, lw=1, arrowstyle="-|>", color="b")

    ax2.set_facecolor('white')
    ax2.add_collection3d(base_plate_shape_3d)
    ax2.add_patch(work)
    art3d.pathpatch_2d_to_3d(work, z=0, zdir="z")
    ax2.add_artist(a), ax2.add_artist(b), ax2.add_artist(c)
    ax2.xaxis.pane.set_edgecolor('black'), ax2.yaxis.pane.set_edgecolor('black'), ax2.zaxis.pane.set_edgecolor('black')
    ax2.set_xlabel('x [mm]'), ax2.set_ylabel('y [mm]'), ax2.set_zlabel('z [mm]')
    ax2.set_xlim(-box_side, box_side), ax2.set_ylim(-box_side, box_side), ax2.set_zlim(0, 0.5*box_side)
    ax2.add_collection3d(grid_footprint_3d)
    ax2.scatter(grid_x, grid_y, grid_z, 'o', s=2, color='black')
    ax2.scatter(grid_x[0][0], grid_y[0][0], grid_z[0][0], '*', s=10, color='green')  # First point


config = load_use_case_config()
