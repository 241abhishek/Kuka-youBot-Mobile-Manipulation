"""
Python script to generate a trajectory using the 
TrajectoryGenerator, NextState and FeedbackControl funtions.

Usage:
    Define inputs in main and run this file as an executable.
"""

from milestone1.next_state import NextState
from milestone2.trajectory_generator import TrajectoryGenerator
from milestone3.feedback_control import FeedbackControl
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
import logging

def main(args=None):
    """
    Main function.
    """
    # set logging level
    logging.basicConfig(level=logging.INFO)
    logging.info("Script started.")
    
    # define inputs for trajectory generation
    Tse_i = np.array([[0, 0, 1, 0],
                      [0, 1, 0, 0.0],
                      [-1, 0, 0, 0.5],
                      [0, 0, 0, 1]])
    Tsc_i = np.array([[1, 0, 0, 1.0],
                      [0, 1, 0, 0.0],
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

    # generate trajectory
    Traj, Traj_SE3 = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s,k)

    # define inputs for feedback control
    curr_config = np.array([0.0, -0.3, 0.2, 0.0, 0.0, 0.9, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
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
    Kp = 2.5
    Ki = 1.2
    timestep = 0.01
    max_vel = 25
    integral_error = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

    # initialize config_list to track robot configuration
    config_1 = np.append(curr_config, 0)
    config_list = [config_1]

    # initialize X_error_list to track twist error 
    X_error_list = [[0,0,0,0,0,0]]

    # loop over all the reference trajectories
    for i in range(len(Traj)-1):
        # calculate current actual end-effector configuration
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

        # call the FeedbackControl function to generate command velocities
        vel, int_err, X_error = FeedbackControl(X, Traj_SE3[i], Traj_SE3[i+1], Kp, Ki, timestep, curr_config, integral_error)

        # append twist error to list
        X_error_list.append(X_error)

        # update integral error
        integral_error = np.add(integral_error,int_err)

        # call the NextState funciton to generate updated configuration
        curr_config = NextState(curr_config, vel, timestep, max_vel)

        # add the current configuration to config_list
        config = np.append(curr_config, Traj[i][-1])
        config_list.append(config)

    # Overwrite csv file
    # uncomment this to overwrite/create a csv file
    logging.info("Generating animation csv file.")
    # np.savetxt("best.csv", np.asarray(np.c_[config_list]), delimiter = ",")
    np.savetxt("overshoot.csv", np.asarray(np.c_[config_list]), delimiter = ",")
    # np.savetxt("newTask.csv", np.asarray(np.c_[config_list]), delimiter = ",")

    logging.info("Writing error plot data.")
    X_error_list = np.asarray(X_error_list)
    t = np.linspace(0,len(config_list)*timestep,len(config_list))
    plt.plot(t,X_error_list[:,0])
    plt.plot(t,X_error_list[:,1])
    plt.plot(t,X_error_list[:,2])
    plt.plot(t,X_error_list[:,3])
    plt.plot(t,X_error_list[:,4])
    plt.plot(t,X_error_list[:,5])
    plt.title("Error Plot")
    plt.xlabel("Time")
    plt.ylabel("Error")
    plt.legend([r'$Xerr_1$',r'$Xerr_2$',r'$Xerr_3$',r'$Xerr_4$',r'$Xerr_5$',r'$Xerr_6$'])
    plt.show()

    # Save data to CSV file
    data_to_save = np.column_stack((t, X_error_list))
    np.savetxt('error_data.csv', data_to_save, delimiter=',')

    logging.info("Done.")

if __name__ == "__main__":
    main()