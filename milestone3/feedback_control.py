import numpy as np
import modern_robotics as mr

def FeedbackControl(X, X_d, X_d_next, Kp, Ki, timestep):
    integral_error = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    X_err = mr.se3ToVec(mr.TransInv(X)@X_d)
    # print(X_err)
    integral_error = integral_error + X_err*timestep
    # print(integral_error)
    V_d = (1/timestep)*mr.se3ToVec(mr.TransInv(X_d)@X_d_next)
    # print(V_d)
    V = mr.Adjoint(mr.TransInv(X)@X_d)@V_d+ Kp*X_err + Ki*integral_error
    print(V)
    return

def main(args=None):
    """
    Main function.
    """
    # define inputs
    robot_config = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.2, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    X = np.array([[0.170, 0, 0.985, 0.387],
                    [0, 1, 0, 0],
                    [-0.985, 0, 0.170, 0.570],
                    [0, 0, 0, 1]])
    X_d = np.array([[0, 0, 1, 0.5],
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0.5],
                    [0, 0, 0, 1]])
    X_d_next = np.array([[0, 0, 1, 0.6],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.3],
                        [0, 0, 0, 1]])
    Kp = 1
    Ki = 0
    timestep = 0.01
    FeedbackControl(X, X_d, X_d_next, Kp, Ki, timestep)


if __name__ == "__main__":
    main()