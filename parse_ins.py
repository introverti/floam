from cmath import tan
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

    # ex_matrix = np.asarray([[-0.04043280672664568, 0.9987881997133227, -0.02805922760208665, -1.888119849631524],
    #                         [-0.999182160582223, -0.04040434709786311,
    #                             0.001580730791857529, 0.05351019136297286],
    #                         [0.0004451004904981459, 0.02810019304231532,
    #                             0.9996050125107107, -2.045108655029019],
    #                         [0, 0, 0, 1]])

    # ins_folder = "/home/xavier/repos/data/ins"
    # base_addr = os.listdir(ins_folder)
    # base_addr.sort()
    # tank = []
    # first_line = np.loadtxt(os.path.join(ins_folder, base_addr[0]))
    # origin_t = np.array([first_line[0], first_line[1],
    #                     first_line[2]]).reshape(3, 1)
    # origin_r = np.array([first_line[8], first_line[9], first_line[10]])
    # row4 = np.array([0, 0, 0, 1]).reshape(1, 4)
    # oring_R = R.from_euler("xyz", origin_r)
    # origin_rt = np.hstack((oring_R.as_matrix(), origin_t))
    # origin_rt = np.vstack((origin_rt, row4))
    # inv_origin_rt = np.linalg.inv(origin_rt)

    # for bb in base_addr:
    #     ftxt = np.loadtxt(os.path.join(ins_folder, bb))
    #     timestamp = bb.replace('_ins.txt', '')

    #     current_t = np.array([ftxt[0], ftxt[1], ftxt[2]]).reshape(3, 1)
    #     current_r = np.array([ftxt[8], ftxt[9], ftxt[10]])
    #     current_R = R.from_euler("xyz", current_r)
    #     current_rt = np.hstack((current_R.as_matrix(), current_t))
    #     current_rt = np.vstack((current_rt, row4))

    #     to_origin_pose = inv_origin_rt@current_rt

    #     to_lidar_pose = ex_matrix@to_origin_pose@np.linalg.inv(ex_matrix)

    #     imu_R = R.from_matrix(to_lidar_pose[0:3, 0:3])
    #     imu_quat = imu_R.as_quat()

    #     tank.append([int(timestamp), to_lidar_pose[0][3], to_lidar_pose[1][3], to_lidar_pose[2][3],
    #                  imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]])
    # np_tank = np.array(tank)
    # np.savetxt("imu_pose.txt", np_tank)

    ins_folder = "/home/xavier/repos/floam/data/left_25"
    base_addr = os.listdir(ins_folder)
    base_addr.sort()
    print (base_addr)