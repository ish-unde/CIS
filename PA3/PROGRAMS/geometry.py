import numpy as np
from registration import find_rigid_transform

def closest_point_triange(P, A, B, C):
    """ find the closest point on the triangle ABC to point P"""

    # point A
    AB = B - A
    AC = C - A
    AP = P - A

    d1 = np.dot(AB, AP)
    d2 = np.dot(AC, AP)

    if d1 <= 0.0 and d2 <= 0.0:
        return A, np.linalg.norm(P - A)
    
    # point B
    BP = P - B
    d3 = np.dot(AB, BP)
    d4 = np.dot(AC, BP)

    if d3 >= 0.0 and d4 <= d3:
        return B, np.linalg.norm(P - B)
    
    # point C
    CP = P - C
    d5 = np.dot(AB, CP)
    d6 = np.dot(AC, CP)

    if d6 >= 0.0 and d5 <= d6:
        return C, np.linalg.norm(P - C)
    

    # edge AB
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        closest = A + v * AB
        return closest, np.linalg.norm(P - closest)
    
    # edge AC
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        closest = A + w * AC
        return closest, np.linalg.norm(P - closest)
    
    # edge BC
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        closest = B + w * (C - B)
        return closest, np.linalg.norm(P - closest)
    
    # p is inside the face region of the triangle
    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    closest = A + AB * v + AC * w
    return closest, np.linalg.norm(P - closest)


def closest_point_mesh(P, vertices, triangles):
    """ find the closest point on the mesh defined by vertices and faces to point P"""

    min_distance = float('inf')
    closest_point = None

    for triangle in triangles:
        A = vertices[triangle[0]]
        B = vertices[triangle[1]]
        C = vertices[triangle[2]]

        point, distance = closest_point_triange(P, A, B, C)

        if distance < min_distance:
            min_distance = distance
            closest_point = point

    return closest_point, min_distance

def compute_transform_from_markers(body_markers, tracker_markers):
    """
    Compute the homogeneous transformation matrix from body coordinates to tracker coordinates
    """
    # numpy arrays from file 
    body_pts = np.array(body_markers)
    tracker_pts = np.array(tracker_markers)
    
    # Find rigid transformation
    R, t = find_rigid_transform(body_pts, tracker_pts)
    
    # Create the homogeneous transformation matrix
    F = np.eye(4)
    F[:3, :3] = R
    F[:3, 3] = t.flatten()
    
    return F




"""
This closest point on triangle algorithm is based on Christer Ericson's
 method from Real-Time Collision Detection, which partitions the space 
 into Voronoi regions corresponding to the triangle's vertices, edges, a
 nd interior. By testing the barycentric coordinates and dot products of 
 vectors from the point to the triangle's vertices, it efficiently determines
   which region contains the point and computes the closest point 
   accordingly; either a vertex, a point on an edge, or a point inside the triangle.

   Ericson, C. (2005). Real-Time Collision Detection. Morgan Kaufmann Publishers. pp. 141-142.
"""
