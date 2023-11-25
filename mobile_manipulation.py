from milestone1.next_state import NextState
from milestone2.trajectory_generator import TrajectoryGenerator
from milestone3.feedback_control import FeedbackControl
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt

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

    Traj, Traj_SE3 = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k)

    curr_config = np.array([0.0, 0.0, 0.0, 0.0, 1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    T_b0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])
    M = np.array([[1,0,0,0.033],
                     [0,1,0,0],
                     [0,0,1,0.6546],
                     [0,0,0,1]])
    B_list_arm = np.array([[0,0,0,0,0],
                        [0,-1,-1,-1,0],
                        [1,0,0,0,1],
                        [0,-0.5076,-0.3526,-0.2176,0],
                        [0.033,0,0,0,0],
                        [0,0,0,0,0]])
    
    Kp = 2
    Ki = 0
    timestep = 0.01
    max_vel = 10000000000000000000000000000000
    config_1 = np.append(curr_config, 0)
    config_list = [config_1]
    integral_error = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    X_error_list = []

    for i in range(len(Traj)-1):
        phi = curr_config[0]
        x = curr_config[1]
        y = curr_config[2]

        Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                        [np.sin(phi), np.cos(phi), 0, y],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
        thetalist_arm = curr_config[3:8]
        T_0e = mr.FKinBody(M,B_list_arm,thetalist_arm)
        X = Tsb@T_b0@T_0e

        vel, int_err, X_error = FeedbackControl(X, Traj_SE3[i], Traj_SE3[i+1], Kp, Ki, timestep, curr_config, integral_error)
        X_error_list.append(X_error)
        integral_error = np.add(integral_error,int_err)
        curr_config = NextState(curr_config, vel, timestep, max_vel)
        config = np.append(curr_config, Traj[i][-1])
        config_list.append(config)
    
    # print(config_list)
    # print(config_list)
    # Overwrite csv file
    # uncomment this to overwrite/create a csv file
    np.savetxt("animation_test.csv", np.asarray(np.c_[config_list]), delimiter = ",")

    X_error_list = np.asarray(X_error_list)
    plt.plot(X_error_list[:,0])
    plt.plot(X_error_list[:,1])
    plt.plot(X_error_list[:,2])
    plt.plot(X_error_list[:,3])
    plt.plot(X_error_list[:,4])
    plt.plot(X_error_list[:,5])
    plt.show()


if __name__ == "__main__":
    main()