import numpy as np
import modern_robotics as mr

def FeedbackControl(X, X_d, X_d_next, Kp, Ki, timestep, curr_config, integral_error):

    X_err = mr.se3ToVec(mr.MatrixLog6((mr.TransInv(X))@X_d))
    
    # integral_error = np.add(integral_error,X_err*timestep)
    # print(integral_error)

    int_err = X_err*timestep

    V_d = mr.se3ToVec((1/timestep)*mr.MatrixLog6((mr.TransInv(X_d))@X_d_next))
    
    V = mr.Adjoint((mr.TransInv(X))@X_d)@V_d + Kp*X_err + Ki*integral_error

    B_list_arm = np.array([[0,0,0,0,0],
                        [0,-1,-1,-1,0],
                        [1,0,0,0,1],
                        [0,-0.5076,-0.3526,-0.2176,0],
                        [0.033,0,0,0,0],
                        [0,0,0,0,0]])
    thetalist_arm = curr_config[3:8]
    Jb = mr.JacobianBody(B_list_arm, thetalist_arm)

    # define matrix F to compute chassis planar twist Vb
    r = 0.0475
    l = 0.235
    w = 0.15
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])

    F6 = np.zeros(shape=(6,4))

    F6[2:-1,:] = F

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
    Je = np.zeros(shape=(6,9))
    Je[:,:4] = J_base
    Je[:,4:] = Jb
    # print(V)
    vel = np.linalg.pinv(Je)@V
    return vel, int_err, X_err

def main(args=None):
    """
    Main function.
    """
    # define inputs
    curr_config = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.2, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
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
    Kp = 0
    Ki = 0
    timestep = 0.01
    integral_error = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

    Vel,int_err, X_error = FeedbackControl(X, X_d, X_d_next, Kp, Ki, timestep, curr_config, integral_error)
    # print(np.around(Vel,1))

if __name__ == "__main__":
    main()