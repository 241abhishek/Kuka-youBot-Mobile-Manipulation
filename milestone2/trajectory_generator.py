import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k):
    def TrajectoryParser(traj, Tse_mat, gripper_state):
        Tse_list = []
        for i in range(len(traj)):
            for j in range(3):
                for k in range(3):
                    Tse_list.append(traj[i][j][k])
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

    waypoint1 = Tsc_i@Tce_s
    waypoint2 = Tsc_i@Tce_g
    waypoint3 = Tsc_i@Tce_g
    waypoint4 = Tsc_i@Tce_s
    waypoint5 = Tsc_f@Tce_s
    waypoint6 = Tsc_f@Tce_g
    waypoint7 = Tsc_f@Tce_g
    waypoint8 = Tsc_f@Tce_s

    traj_1 = mr.ScrewTrajectory(Tse_i,waypoint1,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_1, Tse_mat, 0)

    traj_2 = mr.ScrewTrajectory(waypoint1,waypoint2,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_2, Tse_mat, 0)

    traj_3 = mr.ScrewTrajectory(waypoint2,waypoint3,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_3, Tse_mat, 1)

    traj_4 = mr.ScrewTrajectory(waypoint3,waypoint4,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_4, Tse_mat, 1)

    traj_5 = mr.ScrewTrajectory(waypoint4,waypoint5,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_5, Tse_mat, 1)

    traj_6 = mr.ScrewTrajectory(waypoint5,waypoint6,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_6, Tse_mat, 1)

    traj_7 = mr.ScrewTrajectory(waypoint6,waypoint7,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_7, Tse_mat, 0)

    traj_8 = mr.ScrewTrajectory(waypoint7,waypoint8,2,2/0.01,3)
    Tse_mat = TrajectoryParser(traj_8, Tse_mat, 0)

    # Overwrite csv file
    # uncomment this to overwrite/create a csv file
    np.savetxt("Tse_mat.csv", np.asarray(np.c_[Tse_mat]), delimiter = ",")    

    return Tse_mat

def main(args=None):
    """
    Main function.
    """
    # define initial conditions
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

    # # Overwrite csv file
    # # uncomment this to overwrite/create a csv file
    # np.savetxt("thetamat.csv", np.asarray(np.c_[output]), delimiter = ",")


if __name__ == "__main__":
    main()