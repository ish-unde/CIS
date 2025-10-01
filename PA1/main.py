import numpy as np
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from file_io import read_points_from_file
from testing import test_basic_math_operations

def main():
    print("Starting Comprehensive Tests for PA1 Math Library\n")
    
    # tests
    test_basic_math_operations()

    print("All tests completed!")

if __name__ == "__main__":
    main()