import numpy as np

# Functions from Modern robotics by Lynch & Park

def VecToso3( w ):
    """ Returns a 3 x 3 skew symmetric matrix 'so3mat', an element of 
        Lie Algebra so3, corresponding to a given R^3 vector 'w'  """
    if(np.shape(w) == (3,)):
        so3mat = np.array( [ [ 0,   -w[2],  w[1] ], 
                             [ w[2], 0   , -w[0] ],
                             [-w[1], w[0],  0    ] ]  )
        return so3mat
    else:
        raise ValueError('Invalid Input : Input must be R^3 vector')


def VecTose3( xi ):
    """ Returns a 4 x 4 twist matrix 'se3mat', an element of 
        Lie Algebra se3, corresponding to a given R^6 vector 'xi' """
    if(np.shape(xi) == (6,)):
        w = xi[:3]
        v = xi[3:]
        # smae as np.block([ [ VecToso3(w),v.reshape(-1,1) ], [ np.zeros(4) ] ])
        se3mat = np.block([ [np.c_[VecToso3(w),v]], [np.zeros(4)] ])
        return se3mat
    else:
        raise ValueError('Invalid Input : Input must be R^6')


def so3ToVec( so3mat ):
    """ Returns R^3 vector 'w' corresponding to 
        a given 3 x 3 skew symmetric matrix 'so3mat', an element of Lie Algebra so3 """
    if(np.shape(so3mat) == (3,3)):
        w = np.array( [ so3mat[2,1], so3mat[0,2], so3mat[1,0] ] )
        return w
    else:
        raise ValueError('Invalid Input : Input must be so3')


def se3ToVec( se3mat ):
    """ Returns R^6 vector xi = [w,v] corresponding to 
        a given 4 x 4 twist matrix 'se3mat', an element of Lie Algebra se3 """
    if(np.shape(se3mat) == (4,4)):
        xi = np.array( [ se3mat[2,1], se3mat[0,2], se3mat[1,0], se3mat[0,3], se3mat[1,3], se3mat[2,3] ] )
        return xi
    else:
        raise ValueError('Invalid Input : Input must be se3')
  

def TransToRp( T ):
    """ Extracts rotation matrix R and translation p from given rigid transformation matrix T """
    return T[:3,:3], T[:3,3]


def RpToTrans( R,p ):
    """ Builds rigid transformation matrix from given rotation matrix R and translation p """
    return np.block( [ [np.c_[R,p]], [np.array([0,0,0,1])] ] )


def InvTrans( T ):
    """ Returns inverse of the rigid transformation matrix T """
    R,p = TransToRp(T)
    return np.block( [ [ np.c_[ R.T, -R.T.dot(p)] ], [ np.array([0,0,0,1]) ] ] )

# ToDo : Optinal argument for selecting output SO3 when xi is so3
def Exp(xi, th=None):
    """ Exp(xi,th) : Compute the matrix exponential Exp(skew(xi)*th), 
                     xi = [w,v], assume unit w or v (for translation) 
        Exp(xi) : Compute the matrix exponential Exp(skew(xi)) """ 
    if(np.shape(xi) == (6,)):
        w = xi[:3]
        v = xi[3:]
    elif(np.shape(xi) == (3,)):
        w = xi
        v = np.zeros(3)
    else:
        raise ValueError('Invalid Input : Input must be so3 or se3 vector')
    mag = np.linalg.norm(w)

    # Pure translation
    if(mag == 0):
        R = np.eye(3)
        if th is None:
            t = v
        else:
            t = v*th
        T = RpToTrans(R,t)
        return T
    
    # Rotation and screw
    if th is None:
        th = mag
        w /= mag
        v /= mag        

    # Rotation + translation
    R = np.cos(th)*np.eye(3) + np.sin(th)*VecToso3(w) + (1-np.cos(th))*np.outer(w,w)
    t = (np.eye(3)-R) @ (np.cross(w,v)) + np.dot(w,v)*th*w
    T = RpToTrans(R,t)
    return T


# ToDo : Explicit formula for v by using Cramer rule for 3 x 3 matrix
# Optinal argument for selecting output so3 when T is SO3
def Log( T ):
    """ Calculates a se3 twist correspoding to rigid transformation matrix T """
    if(np.shape(T) == (3,3)):
        R = T
        t = np.zeros(3)
    elif(np.shape(T) == (4,4)):
        R,t = TransToRp(T)
    else:
        raise ValueError('Invalid Input : Input must be SO3 or SE3 matrix')

    if( abs( np.arccos((np.trace(R)-1.)/2.) ) < 1e-5):
        xi = np.block( [ np.zeros(3), t ] )
    else:
        th = np.arccos((np.trace(R)-1)/2)
        w = np.array( [ R[2,1]- R[1,2], R[0,2]- R[2,0], R[1,0]- R[0,1] ])
        w = w / (2*np.sin(th))
        A = (np.eye(3)-R) @ VecToso3(w) + np.outer(w,w)*th
        xi = np.block( [w,np.linalg.solve(A,t)] )
        xi *= th
    return xi


def Ad( T, xi=None ):
    """ Ad(T) : Compute 6 x 6 Adjoint transformation matrix from rigid transformation matrix T
        Ad(T, se3) : Ad(T)*xi. Coordinate transformation of the twist xi by T """
    # T = [R,p; 0,1], then Ad(T) = [R,0; skew(p)*R,R]
    R,p = TransToRp(T)
    Adjoint = np.block ( [ [R,np.zeros([3,3])], [VecToso3(p) @ R, R] ] )
    if xi:
        return Adjoint.dot(xi)
    else:
        return Adjoint
    

def dAd( T, tau=None ):
    """ dAd(T) : Transpose of Ad(T)
        dAd(T, tau) : dAd(T)*tau. Coordinate transformation of wrench tau by T """
    # Transpose(Ad(T))*wrench 
    if tau:
        return Ad(T).T @ tau
    else:
        return Ad(T).T


def ad( xi1, xi2=None ):
    """ ad(xi1) : Compute 6 x 6 matrix representing Lie Bracket operation from twist xi1 
        ad(xi1, xi2) : Compute Lie Bracket [xi1,xi2] from twists xi1 and xi2 """
    # xi = [w,v], then ad(xi) = [skew(w),0; skew(v), skew(w)]
    w = xi1[:3]
    v = xi1[3:]
    adjoint = np.block( [ [VecToso3(w),np.zeros([3,3])], [VecToso3(v), VecToso3(w)] ])
    if xi2:
       return adjoint.dot(xi2)
    else:
       return adjoint
   

def dad( xi, tau=None):
    """ dad(xi) : Transpose of ad(xi)
        dad(xi, tau) : Cross product between twist xi and wrench tau """
    # xi = [w,v], then dad(xi) = Transpose(ad(xi))
    if tau:
       return ad(xi).T @ tau
    else:
       return ad(xi).T


# def MatrixExp3( so3mat ):
#     """ Calculates rotation matrix R using matrix exponential of given so3 matrix 'so3mat' """
#     th = np.linalg.norm( so3ToVec(so3mat) )
#     hatw = so3mat/th
#     w = so3ToVec(hatw)
#     R = np.cos(th)*np.eye(3) + np.sin(th)*hatw + (1-np.cos(th))*np.outer(w,w)
#     return R


# def MatrixLog3( R ):
#     """ Calculates exponential coordinate (a so3 matrix) correspoding to rotation matrix R """
#     if(np.allclose(R, np.eye(3))):
#         w = np.zeros(3)
#     else:
#         th = np.arccos((np.trace(R)-1)/2)
#         w = np.array( [ R[2,1]- R[1,2], R[0,2]- R[2,0], R[1,0]- R[0,1] ])
#         w = w / (2*np.sin(th))
#         w *= th
#     return VecToso3(w)


# def MatrixExp6( se3mat ):
#     """ Calculates rigid body transfornmation matrix T using matrix exponential of given se3 matrix 'se3mat' """
#     xi = se3ToVec(se3mat)
#     w = xi[:3]
#     v = xi[3:]
#     th = np.linalg.norm(w)

#     if(th == 0):
#         R = np.eye(3)
#         t = v
#     else:
#         R = MatrixExp3(se3mat[:3,:3])
#         w = w/th
#         v = v/th
#         t = (np.eye(3)-R).dot(np.cross(w,v)) + np.dot(w,v)*th*w
#     T = RpToTrans(R,t)
#     return T


# def MatrixLog6( T ):
#     """ Calculates exponential coordinate (a se3 matrix) correspoding to rigid transformation matrix T """
#     R,t = TransToRp(T)
#     if( abs( np.arccos((np.trace(R)-1.)/2.) ) < 1e-5):
#         xi = np.block( [ np.zeros(3), t ] )
#     else:
#         so3mat = MatrixLog3(R)
#         w = so3ToVec(so3mat)
#         th = np.linalg.norm(w)
#         w = w/th
#         A = np.matmul( (np.eye(3)-R), VecToso3(w) ) + np.outer(w,w)*th
#         xi = np.block( [w,np.linalg.solve(A,t)] )
#         xi *= th
#     return VecTose3(xi)
