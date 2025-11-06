import numpy as np

def read_rigid_body(filename):
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip()]

    n_markers = int(lines[0])

    markers = []

    for i in range(1, n_markers + 1):
        coordinates = list(map(float, lines[i].split()))
        markers.append(coordinates)

    tip_coordinates = list(map(float, lines[n_markers + 1].split()))

    return np.array(markers), np.array(tip_coordinates)

def read_body_mesh(filename):
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip()]

    n_vertices = int(lines[0])
    vertices = []

    for i in range(1, n_vertices + 1):
        coordinates = list(map(float, lines[i].split()))
        vertices.append(coordinates)

    n_triangels = int(lines[n_vertices + 1])
    triangles = []

    for i in range(n_vertices + 2, n_vertices + 2 + n_triangels):
        indices = list(map(int, lines[i].split()))
        triangles.append(indices)

    return np.array(vertices), np.array(triangles)


def read_sample_data(filename):
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip()]

    header = lines[0].split()
    total_markers = int(header[0])
    n_samples = int(header[1])


    samples = []
    curr = 1

    for _ in range(n_samples):
        frame_data = []
        for _ in range(total_markers):
            coordinates = list(map(float, lines[curr].split()))
            frame_data.append(coordinates)
            curr += 1
        samples.append(np.array(frame_data))

    return samples, n_samples


def write_output(filename ):

    return None