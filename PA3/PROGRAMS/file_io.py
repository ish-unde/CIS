import numpy as np

def read_rigid_body(filename):
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip()]

    # The first line has format: "N_markers filename"
    first_line_parts = lines[0].split()
    n_markers = int(first_line_parts[0].replace(',', ''))  # Remove commas
    markers = []

    for i in range(1, n_markers + 1):
        # Remove commas from each coordinate value
        coordinates = list(map(lambda x: float(x.replace(',', '')), lines[i].split()))
        markers.append(coordinates)

    # Remove commas from tip coordinates
    tip_coordinates = list(map(lambda x: float(x.replace(',', '')), lines[n_markers + 1].split()))

    return np.array(markers), np.array(tip_coordinates)

def read_body_mesh(filename):
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip()]

    # First line: "N_vertices filename"
    first_line_parts = lines[0].split()
    n_vertices = int(first_line_parts[0].replace(',', ''))  # Remove commas
    vertices = []

    for i in range(1, n_vertices + 1):
        # Remove commas from vertex coordinates
        coordinates = list(map(lambda x: float(x.replace(',', '')), lines[i].split()))
        vertices.append(coordinates)

    # Line after vertices: "N_triangles filename" 
    triangles_line_parts = lines[n_vertices + 1].split()
    n_triangles = int(triangles_line_parts[0].replace(',', ''))  # Remove commas
    triangles = []

    for i in range(n_vertices + 2, n_vertices + 2 + n_triangles):
        # Remove commas from triangle indices
        indices = list(map(lambda x: int(x.replace(',', '')), lines[i].split()[:3]))
        triangles.append(indices)

    return np.array(vertices), np.array(triangles)

def read_sample_data(filename):
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip()]

    # First line: "total_markers, n_samples, filename" or "total_markers n_samples filename"
    header_parts = lines[0].split()
    
    # Clean each part by removing commas and converting to int
    total_markers = int(header_parts[0].replace(',', ''))
    n_samples = int(header_parts[1].replace(',', ''))
    

    samples = []
    curr = 1

    for sample_idx in range(n_samples):
        frame_data = []
        for marker_idx in range(total_markers):
            if curr >= len(lines):
                print(f"Warning: Expected {total_markers} markers but reached end of file at sample {sample_idx}")
                break
            
            # Remove commas from coordinate values
            clean_line = lines[curr].replace(',', '')
            coordinates = list(map(float, clean_line.split()))
            frame_data.append(coordinates)
            curr += 1
        
        if len(frame_data) == total_markers:
            samples.append(np.array(frame_data))
        else:
            print(f"Warning: Sample {sample_idx} has {len(frame_data)} markers instead of {total_markers}")

    return samples, len(samples)
