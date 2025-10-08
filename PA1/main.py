import sys
import numpy as np
from pathlib import Path
import math_utils_cart
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from math_utils_cart import find_fd, find_fa, compute_expected_C, read_calbody, read_calreadings
from math_utils_cart import read_empivot, read_optpivot, em_tracking, opt_pivot_calibration, write_output, write_registration, write_em_pivot, write_opt_pivot
from file_io import read_points_from_file
# from testing import test_basic_math_operations, test_kabsch_algorithm
import click


@click.command()
@click.option("--data_dir", "-d", default="PA1 Student Data", help="Input data directory")
@click.option("--output_dir", "-o", default="output", help="Output directory")

# registration commands
@click.option("--calbody_file", default="pa1-debug-a-calbody", help="Name of the first input file -- calbody")
@click.option("--calreadings_file", default="pa1-debug-a-calreadings", help="Name of the second input file -- calreadings")
@click.option("--reg_output_file", help="Name of the output registration file")

# empivot commands
@click.option("--em_file", default="pa1-debug-b-empivot", help="Name of EM Pivot file")
@click.option("--em_output_file", help="Name of the em output file")

# optpivot commands
@click.option("--opt_file", default="pa1-debug-g-optpivot", help="Name of OPT Pivot file")
@click.option("--opt_output_file", help="Name of the opt output file")


@click.option("--combine", is_flag=True, help="Combine all outputs into one file")
@click.option("--reg", help="name of the output registration file")
@click.option("--em", help="name of the em output file")
@click.option("--opt", help="name of the opt output file")
@click.option("--output", help="name of the combined output file")



def main(data_dir, output_dir, calbody_file, calreadings_file, reg_output_file,  em_file, em_output_file, opt_file, opt_output_file, combine, reg, em, opt, output):    

    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()

    cal_path = data_dir / f"{calbody_file}.txt"
    calreadings = data_dir / f"{calreadings_file}.txt"
    em_path = data_dir / f"{em_file}.txt"
    opt_path = data_dir / f"{opt_file}.txt"

    # making output file incase not specified
    if not output_dir.exists():
        output_dir.mkdir()


    # getting output file details and elements
    all_c_expected = None
    n_c = None
    n_frames = None
    em_pivot_post = None
    opt_pivot_post = None


    cal_body = math_utils_cart.read_calbody(str(cal_path))

    cal_read = math_utils_cart.read_calreadings(str(calreadings))

    # empivot = math_utils_cart.read_empivot(    f"{em_path}.txt")

    optpivot = math_utils_cart.read_optpivot(str(opt_path))
    
    
    if calbody_file and calreadings_file and reg_output_file:
        # question 4. a, b, and c
        try:
            d_points, a_points, c_points = cal_body
            fd = find_fd(cal_read, d_points)
            all_fa = find_fa(cal_read, a_points)
            all_c_expected = compute_expected_C(cal_read, fd, all_fa, c_points)

            n_c = len(c_points)
            n_frames = len(cal_read)

            output_path = output_dir / f"{reg_output_file}.txt"
            write_registration(output_path, n_c, n_frames, all_c_expected)
        except BaseException as err:
            print(f"Unexpected {err=}, {type(err)=}")
            raise
        pass
    elif em_file and em_output_file and em_output_file:
        # question 5.
        try:
            em_frames = read_empivot(str(em_path))
            t_g, p_dimple = em_tracking(em_frames)

            em_pivot_post = p_dimple

            output_path = output_dir / f"{em_output_file}.txt"
            write_em_pivot(output_path, em_pivot_post)
        except BaseException as err:
            print(f"Unexpected {err=}, {type(err)=}")
            raise
        pass
    elif opt_file and opt_output_file and opt_output_file:
        # question 6.
        try:
            d_points = cal_body[0]
            t_h, p_dimple = opt_pivot_calibration(optpivot, d_points)

            opt_pivot_post = p_dimple

            output_path = output_dir / f"{opt_output_file}.txt"
            write_opt_pivot(output_path, opt_pivot_post)
        except BaseException as err:
            print(f"Unexpected {err=}, {type(err)=}")
            raise

    if combine and reg and em and opt and output is not None:
        # combine all outputs into one file
        try:
            reg_path = output_dir / f"{reg}.txt"
            em_path = output_dir / f"{em}.txt"
            opt_path = output_dir / f"{opt}.txt"
            output_path = output_dir / f"{output}.txt"
            write_output(reg_path, em_path, opt_path, output_path)
        except BaseException as err:
            print(f"Unexpected {err=}, {type(err)=}")
            raise   
    

if __name__ == "__main__":
    main()





""" Solutions to 4a, 4b, 4c, 4d, 5, and 6

        To generated the c_expected for points a, run:
        python main.py --calbody_file pa1-debug-a-calbody --calreadings_file pa1-debug-a-calreadings --reg_output_file a_registration


        To generate the EM pivot for points a, run:
        python main.py --em_file pa1-debug-a-empivot --em_output_file a_em_pivot

        To generate the Opt pivot for points a, run:
        python main.py --opt_file pa1-debug-a-optpivot --opt_output_file a_opt_pivot 

        To generate all outputs for points a, run:
        python main.py --combine --reg a_registration --em a_em_pivot --opt a_opt_pivot --output a_all_outputs 

        * --name pa1-debug-a-calbody 
    """ 