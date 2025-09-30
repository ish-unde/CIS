from math_utils_cart import Point3D

def read_points_from_file(filename, num_points, start_line=0):
    """Read 3D points from a file"""
    points = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        for i in range(start_line, start_line + num_points):
            coords = list(map(float, lines[i].strip().split(',')))
            points.append(Point3D(coords[0], coords[1], coords[2]))
    return points