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