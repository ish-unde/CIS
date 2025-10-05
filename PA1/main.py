import numpy as np
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from file_io import read_points_from_file
from testing import test_basic_math_operations, test_kabsch_algorithm

def main():
    print("Tests for PA1 \n")
    #tests 
    test_basic_math_operations()
    test_kabsch_algorithm()

    print("tests completed")

if __name__ == "__main__":
    main()

# part 4a. 

def find_fd(frames):
    """ 
    computing the transformation FD between optical tracker and EM tracker coordinate
    """

    # read all the d_j points from file

    d_points = read_points_from_file("data/points_d.txt") # not correct
    d_points = [Point3D(*point) for point in d_points]

    optical_A = frames[1].points

    optical_D = frames[0].points

    F_d = find_rigid_transform(optical_D, d_points)

    return F_d


# helper to read files 
def read_frames_from_file(filename):
    frames = []

    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_d_points = int(header_parts[0])
        num_a_points = int(header_parts[1])
        num_c_points = int(header_parts[3])
        num_frames = int(header_parts[2])


        for _ in range(num_frames):
            d_points = []
            a_points = []
            c_points = []
            for _ in range(num_d_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                d_points.append(Point3D(x, y, z))
            for _ in range(num_a_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                a_points.append(Point3D(x, y, z))
            for _ in range(num_c_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                c_points.append(Point3D(x, y, z))
            frames.append(Frame(d_points, a_points, c_points))
    return frames 

