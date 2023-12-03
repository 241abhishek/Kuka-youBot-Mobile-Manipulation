"""
TrajectoryGenerator

Function to generate desired end effector trajectory.

Input:
    Tse_i -- The initial configuration of the end-effector in the reference trajectory.
    Tsc_i -- The cube's initial configuration.
    Tsc_f -- The cube's desired final configuration.
    Tse_g -- The end-effector's configuration relative to the cube when it is grasping the cube.
    Tce_s -- The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube.
    k -- The number of trajectory reference configurations per 0.01 seconds.

Returns:
    Tse_mat -- Trajectory in Nx13 matrix format.
    Tse_SE3_mat -- Trajectory in NxSE3 format.

Usage:
    Define inputs and call the function. Alternatively, run this file as an executable.
    User can modify inputs inside main.
    Example:
        Tse_i = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])
        Tsc_i = np.array([[1, 0, 0, 1],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])
        Tsc_f = np.array([[0, 1, 0, 0],
                        [-1, 0, 0, -1],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])
        Tce_g = np.array([[np.cos(3*np.pi/4), 0, np.sin(3*np.pi/4), 0],
                        [0, 1, 0, 0],
                        [-np.sin(3*np.pi/4), 0, np.cos(3*np.pi/4), 0],
                        [0, 0,  0, 1]])
        Tce_s = np.array([[np.cos(3*np.pi/4), 0, np.sin(3*np.pi/4), 0],
                        [0, 1, 0, 0],
                        [-np.sin(3*np.pi/4), 0, np.cos(3*np.pi/4), 0.15],
                        [0, 0,  0, 1]])
        k = 1

        Traj_1 = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k)
"""

import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k):
    """Generate end-effector trajectory."""

    def TrajectoryParser(traj, Tse_mat, gripper_state):
        """
        Parse the trajectory generated by the ScrewTrajectory
        function and appends to Tse_mat (Nx13 matrix).
        """
        Tse_list = []
        for i in range(len(traj)):
            for j in range(3):
                for n in range(3):
                    Tse_list.append(traj[i][j][n])
            for p in range(3):
                Tse_list.append(traj[i][p][3])
            if gripper_state == 0:
                Tse_list.append(0)
            elif gripper_state == 1:
                Tse_list.append(1)
            Tse_mat.append(Tse_list)
            Tse_list = []  
        return Tse_mat
    
    Tse_mat = []
    Tse_SE3_mat = []

    waypoint1 = Tsc_i@Tce_s
    waypoint2 = Tsc_i@Tce_g
    waypoint3 = Tsc_i@Tce_g
    waypoint4 = Tsc_i@Tce_s
    waypoint5 = Tsc_f@Tce_s
    waypoint6 = Tsc_f@Tce_g
    waypoint7 = Tsc_f@Tce_g
    waypoint8 = Tsc_f@Tce_s

    traj_1 = mr.ScrewTrajectory(Tse_i,waypoint1,4,4*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_1, Tse_mat, 0)
    for i in range(len(traj_1)):
        Tse_SE3_mat.append(traj_1[i])

    traj_2 = mr.ScrewTrajectory(waypoint1,waypoint2,4,4*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_2, Tse_mat, 0)
    for i in range(len(traj_2)):
        Tse_SE3_mat.append(traj_2[i])

    traj_3 = mr.ScrewTrajectory(waypoint2,waypoint3,2,2*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_3, Tse_mat, 1)
    for i in range(len(traj_3)):
        Tse_SE3_mat.append(traj_3[i])

    traj_4 = mr.ScrewTrajectory(waypoint3,waypoint4,4,4*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_4, Tse_mat, 1)
    for i in range(len(traj_4)):
        Tse_SE3_mat.append(traj_4[i])

    traj_5 = mr.ScrewTrajectory(waypoint4,waypoint5,6,6*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_5, Tse_mat, 1)
    for i in range(len(traj_5)):
        Tse_SE3_mat.append(traj_5[i])

    traj_6 = mr.ScrewTrajectory(waypoint5,waypoint6,4,4*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_6, Tse_mat, 1)
    for i in range(len(traj_6)):
        Tse_SE3_mat.append(traj_6[i])

    traj_7 = mr.ScrewTrajectory(waypoint6,waypoint7,2,2*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_7, Tse_mat, 0)
    for i in range(len(traj_7)):
        Tse_SE3_mat.append(traj_7[i])

    traj_8 = mr.ScrewTrajectory(waypoint7,waypoint8,4,4*k/0.01,5)
    Tse_mat = TrajectoryParser(traj_8, Tse_mat, 0)
    for i in range(len(traj_8)):
        Tse_SE3_mat.append(traj_8[i])

    # Overwrite csv file
    # uncomment this to overwrite/create a csv file
    # np.savetxt("Tse_mat.csv", np.asarray(np.c_[Tse_mat]), delimiter = ",")

    return Tse_mat, Tse_SE3_mat

def main(args=None):
    """
    Main function.
    """
    # define inputs
    Tse_i = np.array([[0, 0, 1, 0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0.5],
                      [0, 0, 0, 1]])
    Tsc_i = np.array([[1, 0, 0, 1],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])
    Tsc_f = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])
    Tce_g = np.array([[np.cos(3*np.pi/4), 0, np.sin(3*np.pi/4), 0],
                      [0, 1, 0, 0],
                      [-np.sin(3*np.pi/4), 0, np.cos(3*np.pi/4), 0],
                      [0, 0,  0, 1]])
    Tce_s = np.array([[np.cos(3*np.pi/4), 0, np.sin(3*np.pi/4), 0],
                      [0, 1, 0, 0],
                      [-np.sin(3*np.pi/4), 0, np.cos(3*np.pi/4), 0.15],
                      [0, 0,  0, 1]])
    k = 1

    Traj_1, Traj_SE3_1 = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k)
    # for i in range(len(Traj_SE3_1)):
    #     print(Traj_SE3_1[i])

if __name__ == "__main__":
    main()