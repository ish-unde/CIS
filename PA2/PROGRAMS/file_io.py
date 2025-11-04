from math_utils_cart import Point3D, CalibrationFrame, EmPivotFrame, OptPivotFrame

def read_calreadings(filename):
    frames = []

    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_d_points = int(header_parts[0])
        num_a_points = int(header_parts[1])
        num_c_points = int(header_parts[2])
        num_frames = int(header_parts[3])
        


        for _ in range(num_frames):
            D_j = []
            A_j = []
            C_j = []
            for _ in range(num_d_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                D_j.append(Point3D(x, y, z))

            for _ in range(num_a_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                A_j.append(Point3D(x, y, z))

            for _ in range(num_c_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                C_j.append(Point3D(x, y, z))

            frames.append(CalibrationFrame(D_j, A_j, C_j))
    return frames 

def read_calbody(filename):

    d = []
    a = []
    c = []

    with open(filename, 'r') as file:
        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]
        num_d_points = int(header_parts[0])
        num_a_points = int(header_parts[1])
        num_c_points = int(header_parts[2])

        for _ in range(num_d_points):
            line = file.readline().strip()
            x, y, z = map(float, line.split(','))
            d.append(Point3D(x, y, z))


        for _ in range(num_a_points):
            line = file.readline().strip()
            x, y, z = map(float, line.split(','))
            a.append(Point3D(x, y, z))


        for _ in range(num_c_points):
            line = file.readline().strip()
            x, y, z = map(float, line.split(','))
            c.append(Point3D(x, y, z))

    return d, a, c        

def read_empivot(filename):
    frames = []


    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_g_points = int(header_parts[0])
        num_frames = int(header_parts[1])
        

        for _ in range(num_frames):
            g_j = []
            for _ in range(num_g_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                g_j.append(Point3D(x, y, z))
            frames.append(EmPivotFrame(g_j))
    return frames 

def read_optpivot(filename):
    frames = []

    with open(filename, 'r') as file:
        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]
        num_d_points = int(header_parts[0])
        num_h_points = int(header_parts[1])
        num_frames = int(header_parts[2])



        for _ in range(num_frames):
            h_points = []
            d_points = []


            # reading d points
            for _ in range(num_d_points):

                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                d_points.append(Point3D(x, y, z))

            # reading h points

            for _ in range(num_h_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                h_points.append(Point3D(x, y, z))

            frames.append(OptPivotFrame(d_points, h_points))

    return frames


# added for P2
def read_ct_fiducials(filename):
    """
    Read CT fiducials from NAME-CT-FIDUCIALS.TXT
    """
    points = []
    
    with open(filename, 'r') as file:
        # First line: N_B, filename
        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]
        N_B = int(header_parts[0])
        
        # Next N_B lines: b_x,i, b_y,i, b_z,i
        for _ in range(N_B):
            line = file.readline().strip()
            coords = [coord.strip() for coord in line.split(',') if coord.strip()]
            if len(coords) == 3:
                x, y, z = map(float, coords)
                points.append(Point3D(x, y, z))
    
    return points

# added for P2
def read_em_fiducials(filename):
    """describes frames of data in which the probe is in contact with the corrrresponding CT fiducial points"""

    frames = []


    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_g_points = int(header_parts[0])
        num_frames = int(header_parts[1])
        

        for _ in range(num_frames):
            g_j = []
            for _ in range(num_g_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                g_j.append(Point3D(x, y, z))
            frames.append(EmPivotFrame(g_j))
    return frames 

# added for P2
def read_em_nav(filename):
    """describes frames of data definding test points. 
    
    note: we are to find the corresponding positions of the probe tip wrt the CT coordinates
    """

    frames = []


    with open(filename, 'r') as file:

        header = file.readline().strip()
        header_parts = [part.strip() for part in header.split(',')]

        num_g_points = int(header_parts[0])
        num_frames = int(header_parts[1])
        

        for _ in range(num_frames):
            g_j = []
            for _ in range(num_g_points):
                line = file.readline().strip()
                x, y, z = map(float, line.split(','))
                g_j.append(Point3D(x, y, z))
            frames.append(EmPivotFrame(g_j))
    return frames 





def write_output1(output1_path, a, b, expected_C_all):
    n_c = a
    n_frames = b

    with open(output1_path, 'w') as file:

        file.write(f"{n_c}, {n_frames}, {output1_path.name}\n")

        for frame_expected in expected_C_all:
            for point3d in frame_expected:
                file.write(f"{point3d.x:.2f}, {point3d.y:.2f}, {point3d.z:.2f}\n")



#added for P2
def write_em_output(output_file, em_frames):
    n_frames = len(em_frames)

    with open(output_file, 'w') as file:
        # line 1: N_frames, NAME-OUTPUT.TXT
        file.write(f"{n_frames}, {output_file}\n")

        for frame in em_frames:
            for v in frame.v_points:
                file.write(f"{v.x:.2f}, {v.y:.2f}, {v.z:.2f}\n")


