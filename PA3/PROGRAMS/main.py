import numpy as np
import os
from file_io import read_rigid_body, read_body_mesh, read_sample_data
from geometry import compute_transform_from_markers, closest_point_mesh


def main():
    data_directory = ".."  
    output_directory = "OUTPUT" 

    # File paths for PA3
    body_a_file = os.path.join(data_directory, "2025 PA345 Student Data", "Problem3-BodyA.txt")
    body_b_file = os.path.join(data_directory, "2025 PA345 Student Data", "Problem3-BodyB.txt")
    mesh_file = os.path.join(data_directory, "2025 PA345 Student Data", "Problem3Mesh.sur")
    sample_file = os.path.join(data_directory, "2025 PA345 Student Data", "PA3-J-Unknown-SampleReadingsTest.txt")
    output_file = os.path.join(output_directory, "PA3-J-Unknown-Output.txt")
    
    
    # input data
    markers_A, tip_A = read_rigid_body(body_a_file)
    markers_B, tip_B = read_rigid_body(body_b_file)
    vertices, triangles = read_body_mesh(mesh_file)
    samples, n_samples = read_sample_data(sample_file)
    
    
    # number of markers per body 
    n_A = len(markers_A)
    n_B = len(markers_B)
    
    # Process samples and write output
    with open(output_file, 'w') as f:
        # header: number_of_samples output_filename
        f.write(f"{n_samples} {os.path.basename(output_file)}\n")
        
        processed_count = 0
        for k in range(n_samples):
            frame_data = samples[k]
            
            # markers for body A and B 
            tracker_markers_A = frame_data[:n_A]
            tracker_markers_B = frame_data[n_A:n_A+n_B]
            
            try:
                # F_A,k and F_B,k
                F_A = compute_transform_from_markers(markers_A, tracker_markers_A)
                F_B = compute_transform_from_markers(markers_B, tracker_markers_B)
                
                # d_k = F_B,k^(-1) * F_A,k * A_tip
                A_tip_homogeneous = np.append(tip_A, 1.0)
                F_A_tip = F_A @ A_tip_homogeneous  
                d_k_homogeneous = np.linalg.inv(F_B) @ F_A_tip  
                d_k = d_k_homogeneous[:3] 
                
                # in PA#3, we assume F_reg = I, so s_k = d_k
                s_k = d_k
                
                # Find closest point on mesh to s_k
                c_k, distance = closest_point_mesh(s_k, vertices, triangles)
                
                # output to file 
                f.write(f"{d_k[0]:8.2f} {d_k[1]:8.2f} {d_k[2]:8.2f} ")
                f.write(f"{c_k[0]:12.2f} {c_k[1]:8.2f} {c_k[2]:8.2f} ")
                f.write(f"{distance:9.3f}\n")
                
                processed_count += 1
                
            except Exception as e:
                print(f"Error processing sample {k}: {e}")
    


if __name__ == "__main__":
    main()