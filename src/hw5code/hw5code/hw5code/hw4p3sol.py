'''hw4p3.py

   This is the solution code for HW4 Problem 3.

   This simply uses NumPy to implement the known forward
   kinematics and Jacobian functions for the 3 DOF robot.

'''

import numpy as np


#
#  Link Lengths
#
l1 = 1
l2 = 1


#
#  Forward Kinematics
#
def fkin(q):
    # Precompute the sin/cos values from 2D joint vector.
    sp  = np.sin(q[0,0])
    cp  = np.cos(q[0,0])
    s1  = np.sin(q[1,0])
    c1  = np.cos(q[1,0])
    s12 = np.sin(q[1,0] + q[2,0])
    c12 = np.cos(q[1,0] + q[2,0])
    
    # Calculate the tip position.
    x = np.array([[-sp * (l1 * c1 + l2 * c12)],
                  [ cp * (l1 * c1 + l2 * c12)],
                  [      (l1 * s1 + l2 * s12)]])

    # Return the tip position as a numpy 3x1 column vector.
    return x


#
#  Jacobian
#
def Jac(q):
    # Precompute the sin/cos values from 2D joint vector.
    sp  = np.sin(q[0,0])
    cp  = np.cos(q[0,0])
    s1  = np.sin(q[1,0])
    c1  = np.cos(q[1,0])
    s12 = np.sin(q[1,0] + q[2,0])
    c12 = np.cos(q[1,0] + q[2,0])
    
    # Calculate the tip position.
    J = np.array([[-cp * (l1*c1+l2*c12),  sp * (l1*s1+l2*s12),  sp * l2*s12],
                  [-sp * (l1*c1+l2*c12), -cp * (l1*s1+l2*s12), -cp * l2*s12],
                  [    0               ,       (l1*c1+l2*c12),       l2*c12]])

    # Return the Jacobian as a numpy 3x3 matrix.
    return J


    
#
#  Main Code
#
def main():
    # Set the joint cooordinates.  Explicity make it a column vector
    # by writing it as a list of lists.
    
    
    q = np.array([[np.radians(0)],
                  [np.radians(85)],
                  [np.radians(0.5)]])
    '''
    q = np.array([[np.radians(0)],
                  [np.radians(44.5)],
                  [np.radians(90)]])
    '''

    x = np.array([[1],[0],[1]]) 
    gamma1 = 0.01
    gamma2 = 0.1 
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)
    J = Jac(q)
    JT = np.transpose(J)
    print("TEST CASE:")
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  J)
    print('qdot\n', np.linalg.inv(J) @ x)
    print('xdot\n', J @ np.linalg.inv(J) @ x)
    Jw1 = np.linalg.inv(JT @ J + gamma1**2 * np.eye(3)) @ JT
    Jw2 = np.linalg.inv(JT @ J + gamma2**2 * np.eye(3)) @ JT
    print('qdot weighted 1\n', Jw1 @ x)
    print('xdot  weighted 1\n', J @ Jw1 @ x)
    print('qdot weighted 2\n', Jw1 @ x)
    print('xdot  weighted 2\n', J @ Jw2 @ x)
    
    U, s, V = np.linalg.svd(J)
    sinv = s
    for i in range(len(sinv)): 
       if sinv[i]**2 >= gamma2**2: sinv[i] = 1/sinv[i] 
       else: sinv[i]=sinv[i]/gamma2**2
    Js = np.linalg.inv(V) @ np.diag(sinv) @ np.linalg.inv(U)
    print('U\n', U)
    print('qdot singular\n', Js @ x)
    print('xdot  singular\n', J @ Js @ x)   

if __name__ == "__main__":
    main()
