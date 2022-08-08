from cmath import tan
from distutils.command.build_scripts import first_line_re
import numpy as np
import os
from scipy.spatial.transform import Rotation as R


if __name__ == "__main__":
    # # ex_matrix = np.asarray([[0.9991756749951659, -0.04054495948729169, 0.002019098345412853, 1.858491680743908],
    # #                         [0.04052170220573666, 0.9991238231985737,
    # #                          0.01046793042653927, 0.09950566013453485],
    # #                         [-0.002441751073342627, -0.01037748414786289,
    # #                          0.9999431712224733, 6.191032885887025],
    # #                         [0.0, 0.0, 0.0, 1.0]])

    ex_matrix = np.asarray([[-0.04043280672664568, 0.9987881997133227, -0.02805922760208665, -1.888119849631524],
                            [-0.999182160582223, -0.04040434709786311,
                                0.001580730791857529, 0.05351019136297286],
                            [0.0004451004904981459, 0.02810019304231532,
                                0.9996050125107107, -2.045108655029019],
                            [0, 0, 0, 1]])
    
    source = np.loadtxt("/home/xavier/repos/floam/data/pose/left_25_imu_abs.txt")
    tank = []
    first_line = source[0]
    origin_t = np.array([first_line[1], first_line[2],
                        first_line[3]]).reshape(3, 1)
    origin_r = np.array([first_line[4], first_line[5], first_line[6], first_line[7]])

    row4 = np.array([0, 0, 0, 1]).reshape(1, 4)
    oring_R = R.from_quat(origin_r)

    origin_rt = np.hstack((oring_R.as_matrix(), origin_t))
    origin_rt = np.vstack((origin_rt, row4))

    inv_origin_rt = np.linalg.inv(origin_rt)

    for bb in source:
        timestamp = bb[0]
        current_t = np.array([bb[1], bb[2], bb[3]]).reshape(3, 1)
        current_r = np.array([bb[4], bb[5], bb[6], bb[7]])
        current_R = R.from_quat(current_r)
        current_rt = np.hstack((current_R.as_matrix(), current_t))
        current_rt = np.vstack((current_rt, row4))

        to_origin_pose = inv_origin_rt@current_rt

        to_lidar_pose = ex_matrix@to_origin_pose@np.linalg.inv(ex_matrix)

        imu_R = R.from_matrix(to_lidar_pose[0:3, 0:3])
        imu_quat = imu_R.as_quat()

        tank.append([timestamp, to_lidar_pose[0][3], to_lidar_pose[1][3], to_lidar_pose[2][3],
                     imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]])
    np_tank = np.array(tank)
    np.savetxt("imu_pose.txt", np_tank)