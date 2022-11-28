'''hw6p4.py

   This is the skeleton code for HW6 Problem 4.

   This explores the redundancy handing while following the previous
   trajectory.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from hw6code.GeneratorNode     import GeneratorNode
from hw6code.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *

#
#   Spline Helper
#
#   We could also the Segments module.  But this seemed quicker and easier?
#
def spline(t, T, p0, pf):
    p = p0 + (pf-p0) * (3*t**2/T**2 - 2*t**3/T**3)
    v =      (pf-p0) * (6*t   /T**2 - 6*t**2/T**3)
    return (p, v)


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        
        self.pl = np.array([ 0.3, 0.5, 0.15]).reshape((-1,1))
        self.pr	= np.array([-0.3, 0.5, 0.15]).reshape((-1,1))
	
        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.plow  = np.array([0.0, 0.5, 0.3]).reshape((-1,1))
        self.phigh = np.array([0.0, 0.5, 0.9]).reshape((-1,1))

        # Initialize the current joint position and chain data.
        self.q = self.q0
        self.chain.setjoints(self.q)

        self.err = np.array([0,0,0,0,0,0]).reshape(-1,1)
        self.lam = 80

    # Declare the joint names.
    def jointnames(self):
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']


    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Decide which phase we are in:
        if t < 3:
            # Approach movement:
            (s0, s0dot) = spline(t, 3, 0, 1)

            pd = self.p0 + (self.pr - self.p0) * s0
            vd =           (self.pr - self.p0) * s0dot

            Rd = Reye()
            wd = np.array([0, 0, 0]).reshape((-1,1))

        else:
            # Cyclic movements, after the first 3s, repeat every 5s.
            t1 = (t-3) % 5.0

            # Pre-compute the path variables.  To show different
            # options, we split into position/orientation path
            # variable and compute using both a sinusoid and splines.
            sp1    =           - np.cos(0.4*np.pi * t1)
            sp1dot = 0.4*np.pi * np.sin(0.4*np.pi * t1)
            sp2    =           - np.cos(0.8*np.pi * t1)
            sp2dot = 0.8*np.pi * np.sin(0.8*np.pi * t1)
	    
            if t1 < 2.5:
                (sR, sRdot) = spline(t1,   2.5, 0,  1)
            else:
                (sR, sRdot) = spline(t1-2.5, 2.5,  1, 0)
           
            pmid = 0.5*(self.pl + self.pr)
            pv = 0.5*(pmid+self.p0*3/4)+ 0.5*(self.p0*3/4 - pmid)*sp2
            vv =                     0.5*(self.p0*3/4 - pmid)*sp2dot

            ph = pmid+ 0.5*(self.pl-self.pr) * sp1 
            vh =       0.5*(self.pl-self.pr) * sp1dot 
            
            pd = pv + ph - pmid
            vd = vv + vh
            
            e = np.array([1/np.sqrt(3),1/np.sqrt(3),-1/np.sqrt(3)]).reshape((-1,1))
            Rd = Rote(e,-2*np.pi/3*sR) 
            wd = e * (-2*np.pi/3 * sRdot)
        
	
        q   = self.q
        err = self.err
        
        # Compute the inverse kinematics
        J = np.vstack((self.chain.Jv(), self.chain.Jw())) 
        U, s, V = np.linalg.svd(J, full_matrices=False)
        sinv = s
        for i in range(len(sinv)): 
            sinv[i]=1/sinv[i]
            
        Js = np.transpose(V) @ np.diag(sinv) @ np.transpose(U)
        
        cr = 5
        t1 = q[0,0]
        t2 = q[1,0]
        repulse = cr * 1/(t1**2+t2**2) * np.array([t1, max((abs(t1),t2)),0,0,0,0,0]).reshape((-1,1))
        qdot2 = repulse
        
        xdot = np.vstack((vd, wd))
        
        
        qdot = Js @ (xdot + self.lam * err) + (np.eye(7) - Js @ J) @ qdot2
        
        
	# Integrate the joint position and update the kin chain data.
        q = q + dt * qdot
        self.chain.setjoints(q)

        # Save the joint value and precompute the task error for next cycle.
        self.q   = q
        self.err = np.vstack((ep(pd, self.chain.ptip()),eR(Rd, self.chain.Rtip())))
        
        # Return the position and velocity as python lists.
        
        return (q.flatten().tolist(), qdot.flatten().tolist())



#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
