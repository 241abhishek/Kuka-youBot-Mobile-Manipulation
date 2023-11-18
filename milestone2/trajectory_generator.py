import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k):
    Tse_mat = []
    Tse_list = []
    waypoint1 = Tsc_i@Tce_s
    traj_1 = mr.ScrewTrajectory(Tse_i,waypoint1,2,2/0.01,3)
    for i in range(len(traj_1)):
        for j in range(3):
            for k in range(3):
                Tse_list.append(traj_1[i][j][k])
        for p in range(3):
            Tse_list.append(traj_1[i][p][3])
        Tse_list.append(0)
        Tse_mat.append(Tse_list)
        Tse_list = []

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
    Tce_g = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
    Tce_s = np.array([[0, 1,  0, 0],
                      [-1, 0,  0, 0],
                      [0, 0, 1, 0.1],
                      [0, 0,  0, 1]])
    k = 1

    Traj_1 = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k)

    # # Overwrite csv file
    # # uncomment this to overwrite/create a csv file
    # np.savetxt("thetamat.csv", np.asarray(np.c_[output]), delimiter = ",")


if __name__ == "__main__":
    main()