# --------------------------------------------------------
# CKim - Porting my robot dynamics class to Python
# --------------------------------------------------------

# Basic imports.
import numpy as np
import lap_us_robot.lap_us_robot.lie_group as lg

class ChunRobot:
    """ Class representing a serial robot manipulator and kinematics and dynamics algorithms methods """
    def __init__(self):
        # Kinematic and dynamic parameters of the robot. This must be set according to the specific robot.
        # 0. Number of joints 'n'
        self.num_joints = 6

        # 1. Set of twists of each joint at initial configuration,
        # described in fixed space csys. 'n' sets of 1 x 6 vector
        self.twist_set = np.zeros(self.num_joints,6)

        # 2. Set of initial configuration of each link's body csys. 
        # with respect to fixed space csys. at initial configuration.
        # (n+1) set of 4 x 4 transformation matrix T.
        # Last element is the initial configuration of the End effector Csys.
        # In srLib, T is stored as 12 element array [T11,T21,T31, T12,T22,T32, T13,T23,T33, T14,T24,T34]
        # Here, creates (n+1) x 4 x 4 array
        self.init_tf_set = np.array([np.eye(4,4)]*(self.num_joints+1))
        
        # 3. Set of spatial inertia of each link, in its body csys.
        # n sets of 6 x 6 spatial inertia matrix G
        # In srLib G is stored as 10 element array with 
        # I[0]=Ixx; I[1]=Iyy; I[2]=Izz; I[3]=Ixy; I[4]=Ixz; I[5]=Iyz; I[6,7,8]=[mcx,mcy,mcz]; I[9] = mass
        # Here, creates (n+1) x 6 x 6 array
        self.inertia_set = np.array([np.eye(6,6)]*(self.num_joints))

        # 4. Gravitational acceleration vector in fixed space csys
        self.gravity = np.array([0,0,0, 0,0,-9.81])

        # Following are state variables, which will be updated during simulation        
        # self.joint_angle = np.zeros(self.num_joints)
        # self.joint_velocity = np.zeros(self.num_joints)
        # self.joint_acceleration = np.zeros(self.num_joints)
        # self.joint_torque = np.zeros(self.num_joints)
        # self.current_tf_set = np.array([np.eye(4,4)]*(self.num_joints+1))
        # self.jacobian_spatial = np.zeros((6,self.num_joints))
        # self.jacobian_body = np.zeros((6,self.num_joints))
        # self.djacobian_spatial = np.zeros((6,self.num_joints))
        # self.djacobian_body = np.zeros((6,self.num_joints))
        # self.mass_matrix = np.zeros((self.num_joints,self.num_joints))
        # self.coriolis_matrix = np.zeros((self.num_joints,self.num_joints))
        # self.coriolis_torque = np.zeros(self.num_joints)
        # self.gravity_torque = np.zeros(self.num_joints)

    def forward_kinematics(self, jAng):
        """ Compute the forward kinematics of the robot given joint angles 
            returns the transformation matix representing end effector configuration """
        T = np.eye(4,4)
        for idx in range(self.num_joints):
            T = T @ lg.Exp(self.twist_set[idx], jAng[idx])
        T = T @ self.init_tf_set[self.num_joints,:,:]
        return T
    
    def jacobian_space(self, jAng):
        """ Compute the space Jacobian of the robot link given joint angles 
            link_idx = -1 (default), means end-effector  
            returns 6 x n space Jacobian matrix of the robots end effector """
        Js = np.zeros((6,self.num_joints))
        Js[:,0] = self.twist_set[0]
        T = np.eye(4,4)
        for idx in range(1, self.num_joints):
            T = T @ lg.Exp(self.twist_set[idx-1], jAng[idx-1])
            Js[:,idx] = lg.Ad(T, self.twist_set[:,idx])
        return Js

    def djacobian_space(self, jAng, jVel):
        """ Compute the derivative of space Jacobian of the robot given joint angles and velocity
            returns 6 x n dJs matrix """
        dJs = np.zeros((6,self.num_joints))
        Js = self.jacobian_space(jAng)
        for i in range(0, self.num_joints):
            for j in range(0, i):
                dJs[:,i] = dJs[:,i] + (jVel[j] * lg.ad(Js[:,j], Js[:,i]))
        return dJs

    def jacobian_body(self, jAng, link_idx=-1):
        """ Compute the body Jacobian of the ith link given joint angles
            link_idx = -1 (default), means end-effector  
            returns 6 x n space Jacobian matrix of the ith link """
        Jb = np.zeros((6,self.num_joints))
        if(link_idx == -1):
            T = lg.InvTrans(self.init_tf_set[self.num_joints,:,:])
            link_idx = self.num_joints - 1
        else:
            T = lg.InvTrans(self.init_tf_set[link_idx,:,:])

        Jb[:,link_idx] = lg.Ad(T, self.twist_set[link_idx])
        for idx in range(link_idx-1, -1, -1):
            T = T @ lg.Exp(self.twist_set[idx+1], -jAng[idx+1])
            Jb[:,idx] = lg.Ad(T, self.twist_set[idx])
        return Jb   
    
    def djacobian_body(self, jAng, jVel, link_idx=-1):
        """ Compute the derivative of body Jacobian of the ith link given joint angles and velocity
            returns 6 x n space Jacobian matrix of the ith link """
        dJb = np.zeros((6,self.num_joints))
        Jb = self.jacobian_body(jAng, link_idx)
        for i in range(0, self.num_joints):
            for j in range(i+1, self.num_joints):
                dJb[:,i] = dJb[:,i] + (jVel[j] * lg.ad(Jb[:,i], Jb[:,j]))
        return dJb

    def mass_matrix(self, jAng):
        """ Compute the mass matrix of the robot given joint angles 
            returns n x n mass matrix """
        M = np.zeros((self.num_joints,self.num_joints))
        # for n in range(self.num_joints):
        #     Gb = self.inertia_set[n,:,:]
        #     Jb = self.jacobian_body(jAng, n)
        #     M = M + (Jb.T @ Gb @ Jb)
        # return M
    
        for n in range(self.num_joints):
            Gb = self.inertia_set[n,:,:]
            Jb = self.jacobian_body(jAng, n)
            for i in range(n+1):
                for j in range(i+1):
                    M[i,j] += Jb[:,i].dot(Gb @ Jb[:,j])
                    M[j,i] = M[i,j]
        return M    


    def coriolis_matrix(self, jAng, jVel):
        """ Compute the Coriolis matrix of the robot given joint angle and velocity 
            returns n x n Coriolis matrix """
        C = np.zeros(self.num_joints,self.num_joints)
        Jset = np.zeros(self.num_joints,6,self.num_joints)             
        for i in range(self.num_joints):
            Jset[i,:,:] = self.jacobian_body(jAng, i)

        # for i in range(self.num_joints):
        #     for j in range(self.num_joints):
        #         for idx in range(max(i,j), self.num_joints):
        #             Gb = self.inertia_set[idx,:,:]
        #             Jj = Jset[idx,:,j]
        #             Ji = Jset[idx,:,i]
        #             C1 = Jj.dot( Gb @ lg.ad(Ji) + lg.ad(Ji).T @ Gb )
        #             C2 = Ji.dot( Gb @ lg.ad(Jj) )
        #             for k in range(idx):
        #                 Jk = Jset[idx,:,k]
        #                 C[i,j] += C1.dot(jVel[k]*Jk)
        #                 if(k>j):
        #                     C[i,j] += C2.dot(jVel[k]*Jk)
        #                 else:
        #                     C[i,j] -= C2.dot(jVel[k]*Jk)

        # C = 0.5*C   
        # return C
    
        for i in range(self.num_joints):
            for j in range(self.num_joints):
                lb = max(i,j)
                for idx in range(lb, self.num_joints):
                    Gb = self.inertia_set[idx,:,:]
                    Jj = Jset[idx,:,j]
                    Ji = Jset[idx,:,i]
                    for k in range(idx+1):
                        Jk = jVel[k]*Jset[idx,:,k]
                        P1 = lg.ad(Ji, Jk) @ (Gb @ Jj)
                        P2 = lg.ad(Ji, Jj) @ (Gb @ Jk)
                        P3 = lg.ad(Jj, Jk) @ (Gb @ Ji)

                        if(k>j):
                            C[i,j] = C[i,j] + P1 + P2 + P3
                        else:
                            C[i,j] = C[i,j] + P1 + P2 - P3
                C[i,j] = 0.5 * C[i,j]
        return C


    def gravity_torque(self, jAng):
        """ Compute the gravity torque of the robot given joint angles 
            returns n x 1 gravity torque vector """
        T = np.eye(4)
        g_torque = np.zeros(self.num_joints)
        for idx in range(self.num_joints):
            T = T @ lg.Exp(self.twist_set[idx], jAng[idx])
            g_body = lg.dAd( lg.InvTrans(T @ self.init_tf_set[idx,:,:]), self.gravity)
            Gb = self.inertia_set[idx,:,:]
            g_wrench = Gb @ g_body
            Jb = self.jacobian_body(jAng, idx)
            g_torque = g_torque -(Jb.T @ g_wrench)
        return g_torque
    
    
    def inverse_dynamics(self, jAng, jVel, jAcc, Fext, grav=None):
        """ Compute the inverse dynamics of the robot given joint angles, velocity, and acceleration
            returns n x 1 joint torque vector """
        TwistInLinkCsys = np.zeros(6,self.num_joints)
        for i in range(self.num_joints):
            xi = self.twist_set[i]
            Mi = self.init_tf_set[i,:,:]
            TwistInLinkCsys[i] = lg.Ad(lg.InvTrans(Mi), xi)

        # Forward iteration
        V = np.zeros(6,self.num_joints)
        Vdot = np.zeros(6,self.num_joints)

        V[:,0] = TwistInLinkCsys[0]*jVel[0]
        if(grav is None):
            Vdot[:,0] = TwistInLinkCsys[0]*jAcc[0] - self.gravity
        else:
            Vdot[:,0] = TwistInLinkCsys[0]*jAcc[0] - grav

        for i in range(1,self.num_joints):
            xi = TwistInLinkCsys[i]
            th = jAng[i]
            dth = jVel[i]
            ddth = jAcc[i]

            Mi = self.init_tf_set[i,:,:]
            Miprev = self.init_tf_set[i-1,:,:]
            T = lg.Exp(-xi,th) @ lg.InvTrans(Mi) @ Miprev
            V[:,i] = xi*dth + lg.Ad(T, V[:,i-1])
            Vdot[:,i] = xi*ddth + lg.Ad(T, Vdot[:,i-1]) + dth*lg.ad(V[:,i], xi)

        # Backward iteration
        F = np.zeros(6,self.num_joints+1)
        F[:,self.num_joints] = -Fext
        Tau = np.zeros(self.num_joints)

        for i in range(self.num_joints-1, -1, -1):
            Gi = self.inertia_set[i,:,:]
            Mi = self.init_tf_set[i,:,:]
            Minext = self.init_tf_set[i+1,:,:]
                
            if(i==self.num_joints-1):
                T = lg.InvTrans(Minext) @ Mi
            else:
                th = jAng[i+1]
                xi = TwistInLinkCsys[i+1]
                T = lg.Exp(-xi,th) @ lg.InvTrans(Minext) @ Mi
            F[:,i] = Gi @ Vdot[:,i] - lg.dad(V[:,i], (Gi @ V[:,i])) + lg.dAd(T, F[:,i+1])
            Tau[i] = np.dot(F[:,i], TwistInLinkCsys[:,i])
        return Tau
    

    def coriolis_torque(self, jAng, jVel):
        """ Compute the Coriolis torque of the robot given joint angle and velocity 
            returns n x 1 Coriolis torque vector """
        # Call inverse dynamics with zero acceleration, Fext and gravity.   
        ctq = self.inverse_dynamics(jAng, jVel, np.zeros(self.num_joints), np.zeros(6), np.zeros(6))
        return ctq


    def forward_dynamics(self, tau, jAng, jVel, Fext):
        """ Compute the forward dynamics of the robot given joint torque, angles, velocity, and external wrench
            returns n x 1 joint acceleration vector """

	    # First calculate torque from Coriolois term. C(theta, dtheta)*dtheta
        C = self.coriolis_torque(jAng, jVel)

        # Second calculate torque from gravity. g(theta)
        G = self.gravity_torque(jAng)
        
        # Calculate Mass matrix M(theta)
        M = self.mass_matrix(jAng,)

        # Calculate torque by external wrench
        Jb = self.jacobian_body(jAng)

        # Solve dynamic equation for jAcc
        jAcc = np.linalg.solve(M, tau + Jb.T @ Fext - C - G)
        return jAcc

