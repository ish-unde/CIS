import sys
import numpy as np
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from file_io import read_points_from_file
from testing import test_basic_math_operations, test_kabsch_algorithm

def main():
    print("Tests for PA1 \n")

    if len(sys.argv) < 2:
        print("Usage: python main.py <input_file>")
        sys.exit(1)

    filename = sys.argv[1]
    frames, d_points, a_points, c_points = read_points_from_file(filename)
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
        all_d_points.extend(frame.d_points)


    F_d = find_rigid_transform(all_d_points, all_optical_D)

    return F_d


# helper to read files 
def read_file(filename):
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
