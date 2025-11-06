import numpy as np

def find_rigid_transform(points_A, points_B):
    """
    Find the rigid transformation that best aligns points_A to points_B
    using slides 4 from CIS Class and Direct Technique to solve for R from 
    K.Arun et al. IEEE PAMI, Vol 9 no 5, September 1987.

    """
    A = np.array([p.to_array() for p in points_A]).T  
    B = np.array([p.to_array() for p in points_B]).T  
    
    # turn into centroid
    centroid_A = np.mean(A, axis=1, keepdims=True)
    centroid_B = np.mean(B, axis=1, keepdims=True)
    
    A_centered = A - centroid_A
    B_centered = B - centroid_B
    
    # compute covariance matrix
    H = A_centered @ B_centered.T
    
    # use SVD
    U, S, Vt = np.linalg.svd(H)

    # ensure right-handed coordinate system
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    
    # rotation
    R = Vt.T @ D @ U.T

    # translation 
    t = centroid_B - R @ centroid_A
    
    return R, t

def compute_transform(body_marker, tracker_markers):
    R, t = find_rigid_transform(body_marker.get_markers(), tracker_markers)

    # creating homogeneous representation
    F = np.eye(4)
    F[:3, :3] = R
    F[:3, 3] = t


