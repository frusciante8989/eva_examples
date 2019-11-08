# Automata Technologies - Andrea Antonello, 2019
import socket
from automata import Eva
from evaUtilities import *

# Connection to robot
host = <user_input>
token = <user_input>
eva = Eva(host, token)
eva.lock_status()

# Connection to camera
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server = <user_input>
port = <user_input>
sock.connect((server, port))

# Use-case parameters
objects = ['C', 'M', 'R']
object_heights = {'C': <user_input>, 'M': <user_input>, 'R': <user_input>}  # object thicknesses [m]
end_effector_length = <user_input>  # length of tool [m]
hover_height = <user_input>  # elevation of idle z axis wrt to the object [m]
surface_height = <user_input>  # elevation of the pickup surface wrt to the robot [m, signed]
joint_cal_zero = <user_input>  # joints @ (0,0) of calibration board
joint_guess = <user_input>  # joints guess for pickup/hover position

with eva.lock():
    while True:
        passed, cam_string = readTcpIp(sock) # String format = ['start', 'pattern', 'x_mm', 'y_mm', 'angle', 'pass']:

        if passed is True and len(cam_string) == 5 and cam_string[0] is 'start':  # Successful string format
            evaVision = EvaVision(eva, cam_string, joint_cal_zero, object_heights[cam_string[1]],
                                  surface_height, end_effector_length)
            xyz = evaVision.locate_object()
            xyz_hover = xyz
            xyz_hover[2] = xyz[2] + abs(hover_height)  # add hover height to Z

            # Compute IK for pickup and hover - special case with head down solution
            success_IK_pickup, joints_IK_pickup = solve_ik_head_down(eva, joint_guess, cam_string[4], xyz)
            success_IK_hover, joints_IK_hover = solve_ik_head_down(eva, joint_guess, cam_string[4], xyz_hover)

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
                # Create the way-points, generate the tool-path, save it and execute it
                waypoint_home = <user_input>  # [rad] home position
                waypoint_drop = <user_input>  # [rad] drop-off position
                waypoint_hover = joints_IK_hover  # hover position right on top of object: offset in height (z_pickup)
                waypoint_pickup = joints_IK_pickup  # pickup position on surface of the object

                toolpath_machine_vision = {
                    "metadata": {
                        "default_velocity": 1,
                        "next_label_id": 5,
                        "analog_modes": {"i0": "voltage", "i1": "voltage", "o0": "voltage", "o1": "voltage"}
                    },
                    "waypoints": [
                        {"label_id": 1, "joints": waypoint_home},  # index 0
                        {"label_id": 2, "joints": waypoint_hover},  # index 1
                        {"label_id": 3, "joints": waypoint_pickup},  # index 2
                        {"label_id": 4, "joints": waypoint_drop},  # index 3
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
