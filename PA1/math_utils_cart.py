import numpy as np
from scipy import linalg


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
        self.matrix = np.array(matrix)
    
    def __matmul__(self, other):
        # matrix multiplication
        if isinstance(other, Rotation):
            return Rotation(self.matrix @ other.matrix)
        elif isinstance(other, Point3D):
            rotated = self.matrix @ other.to_array()
            return Point3D.from_array(rotated)
        return NotImplemented

class Frame:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation  
    
    def transform_point(self, point):
        # F dot p = R * p + t
        rotated = self.rotation @ point
        return Point3D(rotated.x + self.translation.x,
                      rotated.y + self.translation.y, 
                      rotated.z + self.translation.z)
    
    def inverse(self):
        # F^{-1} = (R^T, -R^T * t)
        R_inv = Rotation(self.rotation.matrix.T)  
        t_inv = R_inv @ Point3D(-self.translation.x, 
                               -self.translation.y, 
                               -self.translation.z)
        return Frame(R_inv, t_inv)
    
    def compose(self, other):
        # F1 dot F2
        new_rotation = self.rotation @ other.rotation
        new_translation = self.transform_point(other.translation)
        return Frame(new_rotation, new_translation)
    
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
    
    # rotation
    R = Vt.T @ U.T
    
    # handle case of reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # checked properties of rotation matrix 
    assert np.isclose(np.linalg.det(R), 1.0), "Det(R) is not 1"

    # find translation 
    t = centroid_B - R @ centroid_A
    
    # make frame
    rotation = Rotation(R)
    translation = Point3D(t[0, 0], t[1, 0], t[2, 0])
    
    return Frame(rotation, translation)

def calculate_centroid(points):
    if not points:
        return Point3D(0, 0, 0)
    
    sum_x = sum(p.x for p in points)
    sum_y = sum(p.y for p in points)
    sum_z = sum(p.z for p in points)
    n = len(points)
    
    return Point3D(sum_x/n, sum_y/n, sum_z/n)

def transform_points(frame, points):
    return [frame.transform_point(p) for p in points]

#pivot calibration method   
class PivotCalibration:

    def __init__(self):
        self.tip_position = None
        self.pivot_point = None
        self.residual_error = None

    def calibrate(self, poses):
        """
        Perform pivot calibration using parameter estimation 

        Method taken from class slides "Calibration"
            
        Returns:
            tip_position: estimated tip position in tool coordinates (3,)
            pivot_point: estimated pivot point in world coordinates (3,)
        """
        if len(poses) < 2:
            raise ValueError("Need at least 2 poses for pivot calibration")
    
        n_poses = len(poses)
        
        # setting up the matricies for the poses based on calibrations 
        A = np.zeros((3 * n_poses, 6))
        b = np.zeros(3 * n_poses)
        
        for i, (R, p) in enumerate(poses):
            start_idx = 3 * i
            end_idx = 3 * i + 3
            
            # matrix A
            A[start_idx:end_idx, 0:3] = R  # R_j for p_t
            A[start_idx:end_idx, 3:6] = -np.eye(3)  # -I for p_pivot
            
            # b vector
            b[start_idx:end_idx] = -p
        
        # least squares
        x, residuals, rank, s = linalg.lstsq(A, b)

        # check solution of least squares
        if rank < 6:
            print(f"System is underdetermined, rank is < 6: {rank}")

        # check if valid rotation matrix
        if np.isclose(np.linalg.det(R), 1):
            print("Valid rotation matrix with Det(R) = 1")

        # update points and error 
        self.tip_position = x[0:3]
        self.pivot_point = x[3:6]

        if residuals.size > 0:
            self.residual_error = np.sqrt(np.sum(residuals)  / n_poses)
        else:
            self.residual_error = 0.0
        
        return self.tip_position, self.pivot_point


class CalibrationFrame:
    def __init__(self, D, A, C):
        # points are in 3D
        self.D = D  
        self.A = A  
        self.C = C  



# question 4

def find_fd(frames, d_points):
    """ 
    computing the transformation FD between optical tracker and EM tracker coordinate
    """

    all_optical_D = []
    all_d_points = []

    for frame in frames:
        all_optical_D.extend(frame.D)
        all_d_points.extend(d_points)

    
    F_d = find_rigid_transform(all_d_points, all_optical_D)

    return F_d

def find_fa(frames, a_points):
    """
    computing the transformation FA between optical tracker and EM tracker coordinate
    """
    all_F_a = []

    for frame in frames:
        F_a = find_rigid_transform(a_points, frame.A)
        all_F_a.append(F_a)

    return all_F_a

def compute_expected_C(frames, fd, all_fa, c_points):
    """
    computing the expected C position: C_expercted = FD^-1 * FA * c_j
    """

    all_C_expected = []
    fd_inv = fd.inverse()

    for i, (frame, f_a) in enumerate(zip(frames, all_fa)):
        f_composite = fd_inv.compose(f_a)

        c_set = []

        for c_j in c_points:
            c_expected = f_composite.transform_point(c_j)
            c_set.append(c_expected)

        all_C_expected.append(c_set)

        # need to calculate errors associated with this transformation
        errors = []

        for expected, actual in zip(all_C_expected[i], frame.C):
            error = np.linalg.norm([expected.x - actual.x, expected.y - actual.y, expected.z - actual.z])
            errors.append(error)

        mean_error = np.mean(errors)
        print(mean_error)
  

    return all_C_expected


# helper to read files 
def read_calreadings(filename):
    frames = []

    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_d_points = int(header_parts[0])
        num_a_points = int(header_parts[1])
        num_c_points = int(header_parts[2])
        num_frames = int(header_parts[3])
        


        for _ in range(num_frames):
            D_j = []
            A_j = []
            C_j = []
            for _ in range(num_d_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                D_j.append(Point3D(x, y, z))

            for _ in range(num_a_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                A_j.append(Point3D(x, y, z))

            for _ in range(num_c_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                C_j.append(Point3D(x, y, z))

            frames.append(CalibrationFrame(D_j, A_j, C_j))
    return frames 

def read_calbody(filename):

    d = []
    a = []
    c = []

    with open(filename, 'r') as file:
        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]
        num_d_points = int(header_parts[0])
        num_a_points = int(header_parts[1])
        num_c_points = int(header_parts[2])

        for _ in range(num_d_points):
            line = file.readline().strip()
            x, y, z = map(float, line.split(','))
            d.append(Point3D(x, y, z))


        for _ in range(num_a_points):
            line = file.readline().strip()
            x, y, z = map(float, line.split(','))
            a.append(Point3D(x, y, z))


        for _ in range(num_c_points):
            line = file.readline().strip()
            x, y, z = map(float, line.split(','))
            c.append(Point3D(x, y, z))

    return d, a, c        



# question 5

class EmPivotFrame:
    def __init__(self, g_points):
        self.g_points = g_points  


def em_tracking(frames): 

    #extract Gj from empivot 5a (angela : passing the frames in @1:30am)
    g_frames = frames
    g_frames_first = g_frames[0]

    #get frame[0] to calculate midpoint 5a
    g_midpoint = calculate_centroid(g_frames_first.g_points)  

    #then calculate gj!  5a
    g_j_points = []
    for point in g_frames_first.g_points:
        g_j = []
        g_j = Point3D(point.x - g_midpoint.x, point.y - g_midpoint.y, point.z - g_midpoint.z)
        g_j_points.append(g_j)

    #iterate through all frames to get FG[k] 5b
    F_G_frames = []
    for frame in g_frames: 
        F_Gk = find_rigid_transform(g_j_points, frame.g_points) 
        F_G_frames.append(F_Gk)
    
    #5c, finding P_dimple using pivot calibration
    poses = [] 

    for frame in F_G_frames: 
        R = frame.rotation.matrix
        p = frame.translation.to_array()
        poses.append((R, p))


    pivot = PivotCalibration()
    t_g_array, P_dimple_array = pivot.calibrate(poses)

    t_g = Point3D(t_g_array[0], t_g_array[1], t_g_array[2])
    P_dimple = Point3D(P_dimple_array[0], P_dimple_array[1], P_dimple_array[2])

    return t_g, P_dimple
        


def read_empivot(filename):
    frames = []


    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_g_points = int(header_parts[0])
        num_frames = int(header_parts[1])
        

        for _ in range(num_frames):
            g_j = []
            for _ in range(num_g_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                g_j.append(Point3D(x, y, z))
            frames.append(EmPivotFrame(g_j))
    return frames 


# question 6

class OptPivotFrame: 
    def __init__(self, d_points, h_points):
        self.d_points = d_points  
        self.h_points = h_points 


def read_optpivot(filename):
    frames = []

    with open(filename, 'r') as file:
        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]
        num_d_points = int(header_parts[0])
        num_h_points = int(header_parts[1])
        num_frames = int(header_parts[2])



        for _ in range(num_frames):
            h_points = []
            d_points = []


            # reading d points
            for _ in range(num_d_points):

                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                d_points.append(Point3D(x, y, z))

            # reading h points

            for _ in range(num_h_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                h_points.append(Point3D(x, y, z))

            frames.append(OptPivotFrame(d_points, h_points))

    return frames  

def opt_pivot_calibration(frames, d_points):
    
    opt_frames = frames
    first_frame = opt_frames[0]
    h_points_first = first_frame.h_points
    h_midpoint = calculate_centroid(h_points_first)  
    
    h_j_points = []

    for point in first_frame.h_points:
        h_j = Point3D(point.x - h_midpoint.x, point.y - h_midpoint.y, point.z - h_midpoint.z)
        h_j_points.append(h_j)

    
    poses = []

    for frame in opt_frames:
        D = frame.d_points
        H = frame.h_points

        fd = find_rigid_transform(d_points, D)

        fd_inv = fd.inverse()

        h_em = [fd_inv.transform_point(h) for h in H]

        fh = find_rigid_transform(h_j_points, h_em)


        rotation = fh.rotation.matrix
        translation = fh.translation.to_array() 

        poses.append((rotation, translation))

    pivot = PivotCalibration()
    t_h_array, p_dimple_array = pivot.calibrate(poses)

    t_h = Point3D(t_h_array[0], t_h_array[1], t_h_array[2])

    p_dimple = Point3D(p_dimple_array[0], p_dimple_array[1], p_dimple_array[2])

    return t_h, p_dimple



def write_registration(output_file, N_c, N_frames, all_C_expected):
    with open(output_file, 'w') as file:
        file.write(f"{N_c}, {N_frames}, {output_file}\n")

        for expected in all_C_expected:
            for c in expected:
                file.write(f"{c.x:.2f}, {c.y:.2f}, {c.z:.2f}\n")

def write_em_pivot(output_file, em_post_pivot):
    with open(output_file, 'w') as file:
        file.write(f"{em_post_pivot.x:12.2f}, {em_post_pivot.y:12.2f}, {em_post_pivot.z:12.2f}\n")


def write_opt_pivot(output_file, opt_post_pivot):
    with open(output_file, 'w') as file:
        file.write(f"{opt_post_pivot.x:12.2f}, {opt_post_pivot.y:12.2f}, {opt_post_pivot.z:12.2f}\n")

def write_output(reg_file, em_file, opt_file, output_file):
    n_c = 0
    n_frames = 0

    reg_line = ""
    reg_rest = []

    em_line = ""
    opt_line = ""

    if reg_file.exists():
        try:
            with open(reg_file, 'r') as file:
                reg_line = file.readline().strip()
                reg_rest = file.readlines()
        except Exception as e:
            print(f"Error reading registration file {reg_file}: {e}")

    if em_file.exists():
        try:
            with open(em_file, 'r') as file:
                em_line = file.readline().strip()
        except Exception as e:
            print(f"Error reading EM pivot file {em_file}: {e}")

    if opt_file.exists():
        try:
            with open(opt_file, 'r') as file:
                opt_line = file.readline().strip()
        except Exception as e:
            print(f"Error reading OPT pivot file {opt_file}: {e}")
    
    with open(output_file, 'w') as file:
        # line 1: N_c, N_frames, NAME-OUTPUT.TXT
        if reg_line:
            n_c = int(reg_line.split(',')[0].strip())
            n_frames = int(reg_line.split(',')[1].strip())
            file.write(f"{n_c}, {n_frames}, {output_file}\n")
        else:
            file.write(f"0, 0, {output_file}\n")

        # line 2 : estimated post position with EM probe pivot calibration
        if em_line:
            file.write(f"{em_line}\n")
        else:
            file.write(f"{0.0:12.2f}, {0.0:12.2f}, {0.0:12.2f}\n") # write zeros in place

        # line 3 : estimated post position with optical probe pivot calibration
        if opt_line:
            file.write(f"{opt_line}\n")
        else:
            file.write(f"{0.0:12.2f}, {0.0:12.2f}, {0.0:12.2f}\n") # write zeros in place

        # all lines onward
        for line in reg_rest:
            file.write(line)



                