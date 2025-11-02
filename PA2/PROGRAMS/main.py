import sys
import numpy as np
from pathlib import Path
import math_utils_cart
from file_io import (
    read_calbody, read_calreadings, read_empivot, 
    read_ct_fiducials, read_em_fiducials, read_em_nav,
    write_output2
)
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from math_utils_cart import find_fd, find_fa, compute_expected_C, em_tracking, compute_probe_pose
from distortion import DistortionCorrector
from registration import point_set_registration, transform_point
import click


@click.command()
@click.option("--data_dir", "-d", default="PA2 Student Data", help="Input data directory")
@click.option("--output_dir", "-o", default="output", help="Output directory")

# Required files for PA2
@click.option("--calbody_file", required=True, help="Name of the calbody file")
@click.option("--calreadings_file", required=True, help="Name of the calreadings file")
@click.option("--empivot_file", required=True, help="Name of EM Pivot file")
@click.option("--ct_fiducials_file", required=True, help="Name of CT Fiducials file")
@click.option("--em_fiducials_file", required=True, help="Name of EM Fiducials file")
@click.option("--em_nav_file", required=True, help="Name of EM Navigation file")

# Optional parameters
@click.option("--output_file", help="Name of the output file (default: auto-generated)")
@click.option("--poly_degree", default=3, help="Polynomial degree for distortion correction")
@click.option("--validate", is_flag=True, help="Run validation checks")

def main(data_dir, output_dir, calbody_file, calreadings_file, empivot_file,
         ct_fiducials_file, em_fiducials_file, em_nav_file, output_file, poly_degree, validate):
    
    data_path = Path(data_dir)
    output_path = Path(output_dir)

    # question 1

    cal_path = data_dir / f"{calbody_file}.txt"
    calreadings_path = data_dir / f"{calreadings_file}.txt"
    em_path = data_dir / f"{empivot_file}.txt"
    ct_fiducials_path = data_dir / f"{ct_fiducials_file}.txt"
    em_fiducials_path = data_dir / f"{em_fiducials_file}.txt"
    em_nav_path = data_dir / f"{em_nav_file}.txt"


    if not output_dir.exists():
        output_dir.mkdir()

    calbody_data = read_calbody(cal_path)
    calreadings_data = read_calreadings(calreadings_path)
    empivot_data = read_empivot(em_path)
    ct_fiducials_data = read_ct_fiducials(ct_fiducials_path)
    em_fiducials_data = read_em_fiducials(em_fiducials_path)
    em_nav_data = read_em_nav(em_nav_path)


    d_points, a_points, c_points = calbody_data
    fd = find_fd(d_points, calreadings_data)
    fa = find_fa(a_points, calreadings_data, fd)

    expected_C_all = compute_expected_C(calreadings_data, fd, fa, c_points)

    all_measured_c = []

    for frame in calreadings_data:
        all_measured_c.extend(frame['C'])
    all_measured_c = np.array(all_measured_c)
    expected_C_all = np.array(expected_C_all)

    # question 2: create distorction corrector function

    distortion_corrector = DistortionCorrector(degree = poly_degree)
    distortion_corrector.fit(all_measured_c, expected_C_all)
    corrected_train = distortion_corrector.correct(all_measured_c)
    train_errors = np.linalg.norm(corrected_train - expected_C_all, axis=1)
    mean_train_error = np.mean(train_errors)
    std_train_error = np.std(train_errors)

    # question 3: apply distortion correction to EM navigation data
    corrected_empivot_frames = []
    for frame in empivot_data:
        g_measured = np.array(frame['G'])
        g_corrected = distortion_corrector.correct(g_measured)
        corrected_empivot_frames.append({'G':g_corrected.tolist()})

    t_g_corrected, p_dimple_corrected = em_tracking(corrected_empivot_frames)
    