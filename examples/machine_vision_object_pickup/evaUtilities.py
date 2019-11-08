# Automata Technologies - Andrea Antonello, 2019
import math
import operator
import numpy as np


def _convert_to_float(fl):
    """ This method converts ONLY the numeric values of a string into floats """
    try:
        return float(fl)
    except (ValueError, TypeError):
        return fl


def _wrap_to_pi(angle):
    """ This method wrap the input angle to 360 [deg]
    angle : [deg] """
    ang2pi = angle - (angle // (2 * np.pi)) * 2 * np.pi
    if ang2pi > np.pi or ang2pi < - np.pi:
        ang = ang2pi - np.sign(ang2pi) * 2 * np.pi
        return ang


def _quaternion_multiply(quat1, quat0):
    """ This method performs a standard quaternion multiplication
    quat0 : [qR0, qV0], with qR0 being the real part of the quaternion
    quat1 : [qR1, qV1], with qR1 being the real part of the quaternion """
    w0, x0, y0, z0 = quat0
    w1, x1, y1, z1 = quat1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


def _solve_fk(eva, joints):
    """ This method solves the forward kinematics problem and extract the results directly as an array
    joints : joint angles in [rad]
    pos : cartesian position, with respect to robot's origin [m]
    orient : orientation quaternion of the end effector """
    fk_results = eva.calc_forward_kinematics(joints)
    pos_json = fk_results['position']
    orient_json = fk_results['orientation']
    pos = [pos_json['x'], pos_json['y'], pos_json['z']]
    orient = [orient_json['w'], orient_json['x'], orient_json['y'], orient_json['z']]
    return pos, orient


def solve_ik_head_down(eva, guess, theta, xyz_absolute):
    """ This method solves the inverse kinematics problem for the special case of the end-effector
    pointing downwards, perpendicular to the ground.
    guess : is the IK guess, a 1x6 array of joint angles in [rad]
    theta : angular rotation of axis 6 [deg]
    xyz_absolute : cartesian position, with respect to robot's origin [m] """
    pos = [xyz_absolute[0], xyz_absolute[1], xyz_absolute[2]]  # [m]
    pos_json = {'x': (pos[0]), 'y': (pos[1]), 'z': (pos[2])}  # [m]
    orient_rel = [math.cos(np.deg2rad(theta) / 2), 0, 0, math.sin(np.deg2rad(theta) / 2)]
    orient_abs = _quaternion_multiply([0, 0, 1, 0], orient_rel)
    orient_json = {'w': (orient_abs[0]), 'x': (orient_abs[1]), 'y': (orient_abs[2]), 'z': (orient_abs[3])}
    # Compute IK
    result_ik = eva.calc_inverse_kinematics(guess, pos_json, orient_json)
    success_ik = result_ik['ik']['result']
    joints_ik = result_ik['ik']['joints']
    return success_ik, joints_ik


def _read_tcp_ip(sock, objects):
    """ This method reads and decodes the string sent from the camera """
    result = sock.recv(4000)
    string_read = result.decode('utf-8')
    string_split = string_read.split(",")
    camera_string_raw = list(string_split)
    passed = False
    camera_string = ['']
    if len(camera_string_raw) is not 0:
        if camera_string_raw[0] == 'start' and camera_string_raw[19] == 'end' and len(camera_string_raw) == 20:
            camera_string_raw = [_convert_to_float(fl) for fl in camera_string_raw]
            passes = [camera_string_raw[6], camera_string_raw[12], camera_string_raw[18]]
            scores = [camera_string_raw[5], camera_string_raw[11], camera_string_raw[17]]
            passed_score = [passes[0] * scores[0], passes[1] * scores[1], passes[2] * scores[2]]
            max_index, max_value = max(enumerate(passed_score), key=operator.itemgetter(1))
            select_obj = objects[max_index]
            if max_value > 0:
                passed = True
                # Extract the best matching object from the string
                camera_string = _extract_camera_serial(objects, select_obj, camera_string_raw)
    return passed, camera_string


def _extract_camera_serial(objects, index, camera_string_raw):
    """ This method extracts only the best matching object data from the entire string """
    camera_string = ['', 0, 0, 0, 0, 0]
    if index not in objects:
        print('Wrong object in the list')
    elif index is 'C':
        camera_string[0] = 'start'
        camera_string[1] = camera_string_raw[1]
        camera_string[2] = camera_string_raw[2]
        camera_string[3] = camera_string_raw[3]
        camera_string[4] = camera_string_raw[4]
    elif index is 'M':
        camera_string[0] = 'start'
        camera_string[1] = camera_string_raw[7]
        camera_string[2] = camera_string_raw[8]
        camera_string[3] = camera_string_raw[9]
        camera_string[4] = camera_string_raw[10]
    elif index is 'R':
        camera_string[0] = 'start'
        camera_string[1] = camera_string_raw[13]
        camera_string[2] = camera_string_raw[14]
        camera_string[3] = camera_string_raw[15]
        camera_string[4] = camera_string_raw[16]
    return camera_string


class EvaVision:
    """ This class performs the machine vision operations in order to obtain the object position in Eva's frame """
    def __init__(self, eva, string, cal_zero, obj_height=0.0, surf_height=0.0, ee_length=0.0):
        self.eva = eva
        self.string = string
        self.cal = cal_zero
        self.obj = obj_height
        self.surf = surf_height
        self.ee = ee_length

    def locate_object(self):
        print('Pattern identified is: ', self.string[1])
        # Relative object position in camera frame:
        x_obj_rel_cam = 0.001*self.string[2]  # transform X value from [mm] into [m]
        y_obj_rel_cam = 0.001*self.string[3]  # transform Y value from [mm] into [m]
        # Compute relative object position in Eva's frame:
        # Need to known Eva's frame rotation wrt to camera frame
        # Convention: start from camera frame and rotate of ang [deg] to get to Eva's frame
        ang_cam = 180  # [deg]
        x_obj_rel = np.cos(np.deg2rad(ang_cam)) * x_obj_rel_cam + np.sin(np.deg2rad(ang_cam)) * y_obj_rel_cam  # [m]
        y_obj_rel = -np.sin(np.deg2rad(ang_cam)) * x_obj_rel_cam + np.cos(np.deg2rad(ang_cam)) * y_obj_rel_cam  # [m]
        # Compute absolute object position of calibration board origin in Eva's frame:
        pos_cal = self.eva.calc_forward_kinematics(self.cal)['position']
        # Compute absolute object position by summing the calibration board origin to the relative object position
        x_obj_abs = x_obj_rel + pos_cal['x']  # [m]
        y_obj_abs = y_obj_rel + pos_cal['y']  # [m]
        # Compute absolute value of Z
        z_obj_abs = abs(self.obj) + self.surf + abs(self.ee)
        pos_abs = [x_obj_abs, y_obj_abs, z_obj_abs]
        return pos_abs
