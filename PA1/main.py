import sys
import numpy as np
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from file_io import read_points_from_file
from testing import test_basic_math_operations, test_kabsch_algorithm

def main():
    print("Tests for PA1 \n")

    if len(sys.argv) < 3:
        print("Usage: python main.py <input_file>")
        sys.exit(1)

    filename_calreadings = sys.argv[2]
    frames = read_points_from_file(filename_calreadings)

    filename_calbody = sys.argv[1]
    d_points, a_points, c_points = read_calbody(filename_calbody)

    F_d = find_fd(frames, d_points)
    F_a = find_fa(frames, a_points)
    C_expected = compute_expected_C(frames, F_d, F_a, c_points)
    #tests 
    test_basic_math_operations()
    test_kabsch_algorithm()

    print("tests completed")

if __name__ == "__main__":
    main()

# part 4a. 

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

    for i, frame in enumerate(frames):
        F_a = find_rigid_transform(a_points, frame.A)
        all_F_a.append(F_a)

    # not sure if we have to computer an eror metric here

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


        # need to calculate erros associated with this transformation
        errors = []

        for expected, actual in zip(all_C_expected[i], frame.C):
            error = np.linalg.norm(expected.x - actual.x, expected.y - actual.y, expected.z - actual.z)
            errors.append(error)

        mean_error = np.mean(errors)

    return all_C_expected


# helper to read files 
def read_calreadings(filename):
    frames = []


    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_d_points = int(header_parts[0])
        num_a_points = int(header_parts[1])
        num_c_points = int(header_parts[3])
        num_frames = int(header_parts[2])
        


        for _ in range(num_frames):
            d_j = []
            a_j = []
            c_j = []
            for _ in range(num_d_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                d_j.append(Point3D(x, y, z))
            for _ in range(num_a_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                a_j.append(Point3D(x, y, z))
            for _ in range(num_c_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                c_j.append(Point3D(x, y, z))
            frames.append(Frame(d_j, a_j, c_j))
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