from evasdk import Eva
from evaUtilities import *
from progress.bar import *
from config.config_manager import load_use_case_config
import matplotlib.pyplot as plt
import numpy as np


class EvaGrids:
    def __init__(self, _eva, _plot_on_off):
        names = config['grids']['names']
        self.eva = _eva
        self.plot = _plot_on_off
        self.counter_max = {names[0]: config['grids']['row'][names[0]]*config['grids']['col'][names[0]]-1,
                            names[1]: config['grids']['row'][names[1]]*config['grids']['col'][names[1]]-1}
        # Check that slots in second grid are >= than first grid
        slots = [abs(config['grids']['row'][names[0]]) * abs(config['grids']['col'][names[0]]),
                 abs(config['grids']['row'][names[1]]) * abs(config['grids']['col'][names[1]])]
        if slots[0] > slots[1]:
            raise Exception('Drop-off grid is smaller than pick-up grid')

    @staticmethod
    def _create_grid(obj):
        """Creates 2D grid from YAML configuration file
        grid_x, grid_y: list of lists containing all grid points, in cartesian space.
         Units: [m]"""
        row = abs(config['grids']['row'][obj])
        col = abs(config['grids']['col'][obj])
        row_pitch = abs(config['grids']['row_pitch'][obj])
        col_pitch = abs(config['grids']['col_pitch'][obj])
        x0 = config['grids']['x0'][obj]
        y0 = config['grids']['y0'][obj]
        surface = config['grids']['surface'][obj]
        theta = np.deg2rad(config['grids']['angle'][obj])
        grid_x = [[0 for _i in range(col)] for _j in range(row)]
        grid_y = [[0 for _i in range(col)] for _j in range(row)]
        grid_z = [[0 for _i in range(col)] for _j in range(row)]
        grid_points = []
        for _i in range(row):
            for _j in range(col):
                grid_x[_i][_j] = (np.cos(theta+np.pi/2)*col_pitch*_j+np.sin(theta+np.pi/2)*row_pitch*_i)+x0
                grid_y[_i][_j] = -(-np.sin(theta+np.pi/2)*col_pitch*_j+np.cos(theta+np.pi/2)*row_pitch*_i)+y0
                grid_z[_i][_j] = surface
        for _i in range(row):
            for _j in range(col):
                grid_points.append([grid_x[_i][_j], grid_y[_i][_j], grid_z[_i][_j]])

        return grid_x, grid_y, grid_z, grid_points

    def get_grid_points(self, grids):
        """Creates 2D grid using the _create_grid() method and extract the grid point
        corresponding to the object_name and counter selected. It then solves the IK
        for the pickup point [x, y, z] and the hover point [x, y, z + z_hover] and
        provides the corresponding joint angles"""
        if self.plot:
            fig = plt.figure(figsize=(10, 5))
            plt.style.use('seaborn-pastel')
            ax = [fig.add_subplot(121), fig.add_subplot(122, projection='3d')]
        _joints = {grids[0]: {'pick': [], 'hover': []}, grids[1]: {'pick': [], 'hover': []}}

        for grid_iter in grids:
            obj_h = abs(config['grids']['object'][grid_iter])
            ee_h = abs(config['EVA']['end_effector']['length'])
            hover_h = abs(config['EVA']['hover_height'])
            guess = config['grids']['guess'][grid_iter]
            _grid_x, _grid_y, _grid_z, _grid_points = self._create_grid(grid_iter)

            with ChargingBar('Computing ' + config['grids']['names_verbose'][grid_iter] + ' grid', max=len(_grid_points)) as bar:
                for _counter in range(len(_grid_points)):
                    x = _grid_points[_counter][0]/1000
                    y = _grid_points[_counter][1]/1000
                    z = (_grid_points[_counter][2]+ee_h+obj_h)/1000  # [m]
                    pos_obj = [x, y, z]  # [m]
                    pos_obj_hover = [x, y, z + hover_h]  # [m]
                    extra_angle = config['grids']['angle_pickup'][grid_iter]     # Additional pickup angle
                    success_pick, _joints_pick = solve_ik(self.eva, guess, extra_angle, pos_obj)
                    success_hover, _joints_hover = solve_ik(self.eva, guess, extra_angle, pos_obj_hover)
                    if ('success' not in success_pick) or ('success' not in success_hover):
                        raise Exception('IK error')  # Failed IK. Position not reachable
                    _joints[grid_iter]['pick'].append(_joints_pick)
                    _joints[grid_iter]['hover'].append(_joints_hover)
                    bar.next()
                if self.plot:
                    plot_grids(_grid_x, _grid_y, _grid_z, ax, grid_iter)
        if self.plot:
            plt.show()
        move_eva = input("Please verify the correctness of grid placement before continuing "
                         "(to do this, set plot_on_off variable to True and verify grids placement from the plots).\n"
                         "Proceed (this will move the robot)? yes/no\n")
        if 'yes' not in move_eva:
            raise Exception('\n\n\nScript aborted by user: \n - set the plot_on_off variable to True\n '
                            '- run the script again\n - verify grids placement from the plots ')
        return _joints


if __name__ == "__main__":
    # Load use-case parameters
    config = load_use_case_config()

    # Connection to robot
    host = config['EVA']['comm']['host']
    token = config['EVA']['comm']['token']
    eva = Eva(host, token)

    # Compute grid points and robot joints
    plot_on_off = True
    eva_box = EvaGrids(eva, plot_on_off)
    joints = eva_box.get_grid_points(config['grids']['names'])

    # Go home before starting
    with eva.lock():
        eva.control_go_to(config['joints']['home'])

    while True:
        for counter in range(len(joints[(config['grids']['names'][0])]['pick'])):
            joints_home = config['EVA']['home']
            joints_pick = joints[config['grids']['names'][0]]['pick'][counter]
            joints_drop = joints[config['grids']['names'][1]]['pick'][counter]
            joints_pick_hover = joints[config['grids']['names'][0]]['hover'][counter]
            joints_drop_hover = joints[config['grids']['names'][1]]['hover'][counter]

            # USER DEFINED WAY-POINTS
            joints_operation_A = []
            joints_operation_B = []
            joints_operation_C = []

            tool_path_machine_vision = {
                "metadata": {
                    "version": 2,
                    "default_max_speed": 0.1,
                    "next_label_id": 7,
                    "payload": 0,
                    "analog_modes": {"i0": "voltage", "i1": "voltage", "o0": "voltage", "o1": "voltage"}
                    },
                "waypoints": [
                    {"label_id": 1, "joints": joints_home},
                    {"label_id": 2, "joints": joints_pick_hover},
                    {"label_id": 3, "joints": joints_pick},
                    {"label_id": 4, "joints": joints_operation_A},
                    {"label_id": 5, "joints": joints_drop_hover},
                    {"label_id": 6, "joints": joints_drop},
                ],
                "timeline": [
                    {"type": "home", "waypoint_id": 0},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
                    {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0}, "value": True},
                    {"type": "trajectory", "trajectory": "linear", "waypoint_id": 2},
                    {"type": "wait", "condition": {"type": "time", "duration": 0.2}},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 3},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 4},
                    {"type": "trajectory", "trajectory": "linear", "waypoint_id": 5},
                    {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0}, "value": False},
                    {"type": "trajectory", "trajectory": "linear", "waypoint_id": 4},
                    {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 0},
                ]
            }
            with eva.lock():
                eva.control_wait_for_ready()
                eva.toolpaths_use(tool_path_machine_vision)
                eva.control_run(loop=1, mode="automatic")
