import sys
import numpy as np
from pathlib import Path
from file_io import read_calbody, read_calreadings, read_empivot, read_ct_fiducials, read_em_fiducials, read_em_nav
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid
from math_utils_cart import find_fd, find_fa, compute_expected_C, em_tracking
from distortion import DistortionCorrector, b_j_locations, find_f_reg, navigation_data
# from registration import point_set_registration, transform_point
import click


@click.command()
@click.option("--data_dir", "-d", default="PA2/pa_2_student_data/PA 2 Student Data", help="Input data directory")
@click.option("--output_dir", "-o", default=".", help="Output directory")

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

    print(f"output path: {output_path}")
        
    # question 1

    cal_path = data_path / f"{calbody_file}.txt"
    calreadings_path = data_path / f"{calreadings_file}.txt"
    em_path = data_path / f"{empivot_file}.txt"
    ct_fiducials_path = data_path / f"{ct_fiducials_file}.txt"
    em_fiducials_path = data_path / f"{em_fiducials_file}.txt"
    em_nav_path = data_path / f"{em_nav_file}.txt"

    if not output_path.exists():
        output_path.mkdir()

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

    all_measured_c = []
    for frame in calreadings_data:
        for point3d in frame.C:
            all_measured_c.append([point3d.x, point3d.y, point3d.z])

    all_expected_c = []
    for frame_expected in expected_C_all:
        for point3d in frame_expected:
            all_expected_c.append([point3d.x, point3d.y, point3d.z])
    expected_C_all = np.array(all_expected_c)
    all_measured_c = np.array(all_measured_c)

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
        # FIX: Use attribute access for EmPivotFrame objects
        g_measured_points = frame.g_points  # Get the list of Point3D objects
        
        # Convert Point3D objects to numpy array
        g_measured = np.array([[p.x, p.y, p.z] for p in g_measured_points])
        
        g_corrected = distortion_corrector.correct(g_measured)
        corrected_empivot_frames.append({'G': g_corrected.tolist()})

    t_g_corrected, p_dimple_corrected = em_tracking(corrected_empivot_frames)
    first_frame_corrected = corrected_empivot_frames[0]
    if hasattr(first_frame_corrected, 'g_points'):  # EmPivotFrame object
        first_frame_points = first_frame_corrected.g_points
    else:  # Dictionary
        first_frame_points = [Point3D(p[0], p[1], p[2]) for p in first_frame_corrected['G']]

    # Calculate centroid and create g_j_reference as Point3D objects
    centroid = calculate_centroid(first_frame_points)
    g_j_reference = []
    for point in first_frame_points:
        g_j = Point3D(point.x - centroid.x, point.y - centroid.y, point.z - centroid.z)
        g_j_reference.append(g_j)

    #question 4: compute fiducial coordinates 
    b_j_em = b_j_locations(em_fiducials_data, distortion_corrector, t_g_corrected, g_j_reference)

    #question 5: compute Freg 
    F_reg = find_f_reg(ct_fiducials_data, b_j_em)

    #question 6: process navigation data 
    output_file_path = output_path / "pa2-debug-a-output2.txt"
    tip_positions_ct = navigation_data(em_nav_data, distortion_corrector, t_g_corrected, g_j_reference, F_reg, output_file_path)
        
if __name__ == "__main__":
    main()

    