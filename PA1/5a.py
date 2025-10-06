import numpy as np 
from math_utils_cart import Point3D, Rotation, Frame, find_rigid_transform, calculate_centroid, PivotCalibration, calibrate
import sys 

def main(): 
    print("Main")


if __name__ == "__main__":
    main()


def em_tracking(filename): 
    #extract Gj from empivot 5a
    g_frames = read_file_empivot(filename) 
    g_frames_first = g_frames[0]
    #get frame[0] to calculate midpoint 5a
    g_midpointx, g_midpointy, g_midpointz = calculate_centroid(g_frames_first.points)  
    #then calculate gj!  5a
    g_j_points = []
    for point in g_frames_first.points:
        g_j = []
        g_j = (point.x - g_midpointx, point.y - g_midpointy, point.z - g_midpointz)
        g_j_points.append(g_j)

    #iterate through all frames to get FG[k] 5b
    F_G_frames = []
    for frame in g_frames: 
        F_Gk = find_rigid_transform(frame.points, g_j.points)
        F_G_frames.append(F_Gk)
    
    #5c, finding P_dimple using pivot calibration
    poses = [] 

    for frame in F_G_frames: 
        R = frame[:3, :3]
        p = frame[:3, 3]
        poses.append((R, p))

    t_g, P_dimple = calibrate(poses)

    return t_g, P_dimple
        


def read_file_empivot(filename):
    frames = []


    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_g_points = int(header_parts[0])
        num_frames = int(header_parts[2])
        

        for _ in range(num_frames):
            g_j = []
            for _ in range(num_g_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                g_j.append(Point3D(x, y, z))
            frames.append(Frame(g_j))
    return frames 
