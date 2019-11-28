# Automata Technologies - Dr. Andrea Antonello, 2019
import socket
from automata import Eva
from evaUtilities import *
from config.config_manager import load_use_case_config


if __name__ == "__main__":
    # Load config parameters
    config = load_use_case_config()
    
    # Connection to robot
    host = config['EVA']['comms']['host']
    token = config['EVA']['comms']['token']
    eva = Eva(host, token)

    # Connection to camera
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server = config['TCP']['server']
    port = config['TCP']['port']
    sock.connect((server, port))

    # Use-case parameters to be modified in YAML file ~/config/use_case_config.yaml
    objects = config['objects']['names']  # object names [m]
    obj_heights = config['objects']['heights']  # object thicknesses [m]
    ee_length = config['EVA']['end_effector']['length']  # length of tool [m]
    hover_height = config['EVA']['hover_height']  # elevation of idle z axis wrt to the object [m]
    surf_height = config['EVA']['surface_height']  # elevation of the pickup surface wrt to the robot [m, signed]
    joints_cal_zero = config['waypoints']['joints_cal_zero']  # joints @ (0,0) of calibration board
    joints_guess = config['waypoints']['joints_guess']  # joints guess for pickup/hover position
    joints_home = config['waypoints']['joints_home']  # joints guess for home position
    joints_drop = config['waypoints']['joints_drop']  # joints guess for drop position

    with eva.lock():
        while True:
            passed, cam_string = read_tcp_ip(sock, objects)

            if passed is True and len(cam_string) == 5 and cam_string[0] is 'start':
                obj_name = cam_string[1]
                obj_angle = cam_string[4]
                evaVision = EvaVision(eva, cam_string, joints_cal_zero, obj_heights[obj_name], surf_height, ee_length)
                xyz = evaVision.locate_object()  # object position in Eva's frame [m]
                xyz_hover = xyz
                xyz_hover[2] = xyz[2] + abs(hover_height)  # add hover height to object position's Z [m]

                # Compute IK for pickup and hover - special case with head down solution
                success_IK_pickup, joints_pickup = solve_ik_head_down(eva, joints_guess, obj_angle, xyz)
                success_IK_hover, joints_hover = solve_ik_head_down(eva, joints_guess, obj_angle, xyz_hover)

                # Verify IK success
                if 'success' in success_IK_hover and 'success' in success_IK_pickup:
                    perform_move = True
                    message = 'Successful IK'
                    print(message)
                else:
                    perform_move = False
                    message = 'Failed IK. Position not reachable'
                    print(message)

                if perform_move is True:

                    toolpath_machine_vision = {
                        "metadata": {
                            "default_velocity": 1,
                            "next_label_id": 5,
                            "analog_modes": {"i0": "voltage", "i1": "voltage", "o0": "voltage", "o1": "voltage"}
                        },
                        "waypoints": [
                            {"label_id": 1, "joints": joints_home},
                            {"label_id": 2, "joints": joints_hover},
                            {"label_id": 3, "joints": joints_pickup},
                            {"label_id": 4, "joints": joints_drop},
                        ],
                        "timeline": [
                            {"type": "home", "waypoint_id": 0},
                            {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
                            {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0},
                             "value": True},
                            {"type": "trajectory", "trajectory": "linear", "waypoint_id": 2},
                            {"type": "wait", "condition": {"type": "time", "duration": 500}},
                            {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
                            {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 3},
                            {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0},
                             "value": False},
                            {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 0},
                        ]
                    }
                    eva.control_wait_for_ready()
                    eva.toolpaths_use(toolpath_machine_vision)
                    eva.control_run()
            else:
                print('No object correctly recognized')
