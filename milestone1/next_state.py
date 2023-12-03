"""
NextState

Function to generate an updated congiguration for the youBot
given a set of inputs describing its current configuration.

Input:
    curr_config -- A 12-vector representing the current configuration of the robot.
    vel -- A 9-vector of control velocities.
    timestep -- Time duration for the update.
    max_vel -- Maximum angular velocity of the joints.

Returns:
    updated_config -- A 12-vector representing the updated configuration of the robot.

Usage:
    Define inputs and call the function. Alternatively, run this file as an executable.
    User can modify inputs inside main. Some post-processing is required to chain the 
    outputs of the function into an Nx13 matrix for animation in CoppeliaSim.
        Example:
            curr_config = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            vel = np.array([-15.0, 15.0, 15.0, -15.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            timestep = 0.01
            max_vel = 10
            total_sim_time = 1

            waypoints = [np.concatenate((curr_config,1), axis=None)]
            gripper_state = 1
            for i in range(int(total_sim_time/timestep)):
                curr_config = NextState(curr_config, vel, timestep, max_vel)
                curr_config_grip = np.append(curr_config, gripper_state)
                waypoints.append(curr_config_grip)
"""

import numpy as np
import modern_robotics as mr

def NextState(curr_config, vel, timestep, max_vel):
    """Generate trajectory for a four mecanum wheel robot with a 5 dof robotic arm."""

    # define matrix F to compute chassis planar twist Vb
    r = 0.0475
    l = 0.235
    w = 0.15
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])

    # placing limits on joint veolcities
    for i in range(len(vel)):
        if vel[i] > max_vel:
            vel[i] = max_vel
            # print("limiting")
        if vel[i] < -max_vel:
            vel[i] = -max_vel
            # print("limiting")

 
    # update the joint angles
    updated_arm_angles = curr_config[3:8] + vel[4:]*timestep
    updated_wheel_angles = curr_config[-4:] + vel[:4]*timestep
    wheel_angle_increment = updated_wheel_angles - curr_config[-4:]

    # define chassis planar twist
    Vb = F@wheel_angle_increment

    # define chassis planar twist in 6D
    Vb_6D = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0])

    # T matrix realting new body frame pose to initial
    T_b_bnew = mr.MatrixExp6(mr.VecTose3(Vb_6D))

    # define change in coordinates relative to the body frame - d_qb
    w_bz = Vb[0]
    v_bx = Vb[1]
    v_by = Vb[2]
    d_theata_b = w_bz
    if w_bz < 1e-3:
        d_x_b = v_bx
        d_y_b = v_by
    else:
        d_x_b = v_bx*np.sin(w_bz)+v_by*(np.cos(w_bz)-1)/w_bz
        d_y_b = v_by*np.sin(w_bz)+v_bx*(-np.cos(w_bz)+1)/w_bz
    d_qb = np.array([d_theata_b, d_x_b, d_y_b])

    # transform d_qb to space frame - d_qs
    chassis_angle = curr_config[0]
    rot_mat = np.array([[1, 0, 0], [0, np.cos(chassis_angle), -np.sin(chassis_angle)],[0, np.sin(chassis_angle), np.cos(chassis_angle)]])
    d_qs = rot_mat@d_qb

    # define updated chassis config
    chassis_curr_config = np.array(curr_config[:3])
    chassis_updated_config = chassis_curr_config + d_qs

    # form updated_config list in the right order
    updated_config = []
    for item in chassis_updated_config:
        updated_config.append(item)
    for item in updated_arm_angles:
        updated_config.append(item)
    for item in updated_wheel_angles:
        updated_config.append(item)

    return np.array(updated_config)

def main(args=None):
    """
    Main function.
    """
    # define inputs
    curr_config = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel = np.array([-15.0, 15.0, 15.0, -15.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    timestep = 0.01
    max_vel = 10
    total_sim_time = 1

    # form a Nx13 matrix of waypoints
    waypoints = [np.concatenate((curr_config,1), axis=None)]
    gripper_state = 1
    for i in range(int(total_sim_time/timestep)):
        curr_config = NextState(curr_config, vel, timestep, max_vel)
        curr_config_grip = np.append(curr_config, gripper_state)
        waypoints.append(curr_config_grip)

    # Overwrite csv file
    # uncomment this to overwrite/create a csv file
    # np.savetxt("Chassis_waypoints.csv", np.asarray(np.c_[waypoints]), delimiter = ",")

if __name__ == "__main__":
    main()