import numpy as np

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def to_array(self):
        return np.array([self.x, self.y, self.z])
    
    @staticmethod
    def from_array(arr):
        return Point3D(arr[0], arr[1], arr[2])

class Rotation:
    def __init__(self, matrix):
        # 3x3 rotation matrix
        self.matrix = np.array(matrix)
    
    def __matmul__(self, other):
        # Matrix multiplication for rotations
        if isinstance(other, Rotation):
            return Rotation(self.matrix @ other.matrix)
        elif isinstance(other, Point3D):
            rotated = self.matrix @ other.to_array()
            return Point3D.from_array(rotated)
        return NotImplemented

class Frame:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation  # Point3D
    
    def transform_point(self, point):
        # F • p = R * p + t
        rotated = self.rotation @ point
        return Point3D(rotated.x + self.translation.x,
                      rotated.y + self.translation.y, 
                      rotated.z + self.translation.z)
    
    def inverse(self):
        # F^{-1} = (R^T, -R^T * t)
        R_inv = Rotation(self.rotation.matrix.T)  # Transpose
        t_inv = R_inv @ Point3D(-self.translation.x, 
                               -self.translation.y, 
                               -self.translation.z)
        return Frame(R_inv, t_inv)
    
    def compose(self, other):
        # F1 • F2
        new_rotation = self.rotation @ other.rotation
        new_translation = self.transform_point(other.translation)
        return Frame(new_rotation, new_translation)
    
def find_rigid_transform(points_A, points_B):
    """
    Find the rigid transformation that best aligns points_A to points_B
    using Kabsch algorithm (calculates optimal rotation and tranlsation by centering points, moving to origin 
    and using SVD to find rotation and tranlsation components)
    
    Args:
        points_A: List of Point3D in source coordinate system
        points_B: List of Point3D in target coordinate system
    
    Returns:
        Frame: Transformation that maps points_A to points_B
    """
    # Convert to numpy arrays
    A = np.array([p.to_array() for p in points_A]).T  # 3xN
    B = np.array([p.to_array() for p in points_B]).T  # 3xN
    
    # Center the points
    centroid_A = np.mean(A, axis=1, keepdims=True)
    centroid_B = np.mean(B, axis=1, keepdims=True)
    
    A_centered = A - centroid_A
    B_centered = B - centroid_B
    
    # Compute covariance matrix
    H = A_centered @ B_centered.T
    
    # SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Calculate rotation
    R = Vt.T @ U.T
    
    # Handle reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Calculate translation
    t = centroid_B - R @ centroid_A
    
    # Create Frame object
    rotation = Rotation(R)
    translation = Point3D(t[0, 0], t[1, 0], t[2, 0])
    
    return Frame(rotation, translation)

def calculate_centroid(points):
    """Calculate centroid of a set of points"""
    if not points:
        return Point3D(0, 0, 0)
    
    sum_x = sum(p.x for p in points)
    sum_y = sum(p.y for p in points)
    sum_z = sum(p.z for p in points)
    n = len(points)
    
    return Point3D(sum_x/n, sum_y/n, sum_z/n)

def transform_points(frame, points):
    """Apply frame transformation to a list of points"""
    return [frame.transform_point(p) for p in points]