import sys
import numpy as np
from pathlib import Path
import math_utils_cart
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from math_utils_cart import find_fd, find_fa, compute_C_expected, read_calbody, read_calreadings
from math_utils_cart import read_empivot, read_optpivot, em_tracking, parse_files, opt_pivot_calibration
from file_io import read_points_from_file
from testing import test_basic_math_operations, test_kabsch_algorithm
import click


@click.command()
@click.option("--data_dir", "-d", default="PA1 Student Data", help="Input data directory")
@click.option("--output_dir", "-o", default="output", help="Output directory")
@click.option("--name_1", "-n1", default="pa1-debug-a-calbody", help="Name of the first input file")
@click.option("--name_2", "-n2", default="pa1-debug-a-calreadings", help="Name of the second input file")
@click.option("--name_3", "-n3", default="pa1-debug-b-empivot", help="Name of EM Pivot file")
@click.option("--name_4", "-n4", default="pa1-debug-g-optpivot", help="Name of OPT Pivot file")
@click.option("--output_file", "-of", help="Name of the output file")
@click.option("--output_file1", "-of1", help="Name of the output file")
@click.option("--output_file2", "-of2", help="Name of the output file")



def main(data_dir, output_dir, name_1, name_2, name_3, name_4, output_file, output_file1, output_file2):    

    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()

    cal_path = data_dir / f"{name_1}"
    calreadings = data_dir / f"{name_2}"
    em_path = data_dir / f"{name_3}"
    opt_path = data_dir / f"{name_4}"

    # making output file incase not specified
    if not output_dir.exists():
        output_dir.mkdir()

    cal_body = math_utils_cart.read_calbody(
        f"{cal_path}.txt"
    )

    cal_read = math_utils_cart.read_calreadings(
        f"{calreadings}.txt"
    )

    # empivot = math_utils_cart.read_empivot(    f"{em_path}.txt")

    optpivot = math_utils_cart.read_optpivot(
        f"{opt_path}.txt"
    )
    
    
    if name_1 and name_2 and output_file:
        # run 4. a, b, c

        try:
            d_points, a_points, c_points = cal_body
            frames = cal_read

            fd = find_fd(frames, d_points)

            all_fa = find_fa(frames, a_points)

            all_c_expected = compute_C_expected(fd, all_fa, c_points)

            #write_output(str(output_dir / output_file), len(c_points), len(frames), all_c_expected)

        except BaseException as err:
            print(f"Unexpected {err=}, {type(err)=}")
            raise
    elif name_3 and output_file1 and em_path.exists():
        # question 5.
        try:
            em_frames = read_empivot(str(em_path))
            t_g, p_dimple = em_tracking(em_frames)

        except BaseException as err:
            print(f"Unexpected {err=}, {type(err)=}")
            raise
    elif name_4 and output_file2 and opt_path.exists():
        # question 6.
        try:
            opt_frames = read_optpivot(str(opt_path))
            d_points = cal_body[0]
            t_h, p_dimple = opt_pivot_calibration(frames, d_points)


        except BaseException as err:
            print(f"Unexpected {err=}, {type(err)=}")
            raise


    

if __name__ == "__main__":
    main()
