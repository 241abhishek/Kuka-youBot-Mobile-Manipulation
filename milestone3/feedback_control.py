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
    # print(V)
    return V

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
    V = FeedbackControl(X, X_d, X_d_next, Kp, Ki, timestep)

    B_list_arm = np.array([[0,0,0,0,0],
                           [0,-1,-1,-1,0],
                           [1,0,0,0,1],
                           [0,-0.5076,-0.3526,-0.2176,0],
                           [0.033,0,0,0,0],
                           [0,0,0,0,0]])
    thetalist_arm = robot_config[3:8]
    Jb = mr.JacobianBody(B_list_arm, thetalist_arm)
    # print(Jb)

    # define matrix F to compute chassis planar twist Vb
    r = 0.0475
    l = 0.47/2
    w = 0.3/2
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])
    # print(np.shape(F))
    F6 = np.zeros(shape=(6,4))
    # print(F6)
    F6[2:-1,:] = F
    # print(F6)
    T_b0 = np.array([[1,0,0,0.1662],
                     [0,1,0,0],
                     [0,0,1,0.0026],
                     [0,0,0,1]])
    M = np.array([[1,0,0,0.033],
                     [0,1,0,0],
                     [0,0,1,0.6546],
                     [0,0,0,1]])
    T_0e = mr.FKinBody(M,B_list_arm,thetalist_arm)
    J_base = mr.Adjoint(mr.TransInv(T_0e)@mr.TransInv(T_b0))@F6
    # print(J_base)
    Je = np.zeros(shape=(6,9))
    Je[:,:4] = J_base
    Je[:,4:] = Jb
    # print(np.around(Je,3))

    vel = np.linalg.pinv(Je)@V
    # print(np.around(vel,1))
if __name__ == "__main__":
    main()