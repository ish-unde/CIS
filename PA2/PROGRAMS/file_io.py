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



def write_registration(output_file, N_c, N_frames, all_C_expected):
    with open(output_file, 'w') as file:
        file.write(f"{N_c}, {N_frames}, {output_file}\n")

        for expected in all_C_expected:
            for c in expected:
                file.write(f"{c.x:.2f}, {c.y:.2f}, {c.z:.2f}\n")

def write_em_pivot(output_file, em_post_pivot):
    with open(output_file, 'w') as file:
        file.write(f"{em_post_pivot.x:12.2f}, {em_post_pivot.y:12.2f}, {em_post_pivot.z:12.2f}\n")


def write_opt_pivot(output_file, opt_post_pivot):
    with open(output_file, 'w') as file:
        file.write(f"{opt_post_pivot.x:12.2f}, {opt_post_pivot.y:12.2f}, {opt_post_pivot.z:12.2f}\n")

def write_output(reg_file, em_file, opt_file, output_file):
    n_c = 0
    n_frames = 0

    reg_line = ""
    reg_rest = []

    em_line = ""
    opt_line = ""

    if reg_file.exists():
        try:
            with open(reg_file, 'r') as file:
                reg_line = file.readline().strip()
                reg_rest = file.readlines()
        except Exception as e:
            print(f"Error reading registration file {reg_file}: {e}")

    if em_file.exists():
        try:
            with open(em_file, 'r') as file:
                em_line = file.readline().strip()
        except Exception as e:
            print(f"Error reading EM pivot file {em_file}: {e}")

    if opt_file.exists():
        try:
            with open(opt_file, 'r') as file:
                opt_line = file.readline().strip()
        except Exception as e:
            print(f"Error reading OPT pivot file {opt_file}: {e}")
    
    with open(output_file, 'w') as file:
        # line 1: N_c, N_frames, NAME-OUTPUT.TXT
        if reg_line:
            n_c = int(reg_line.split(',')[0].strip())
            n_frames = int(reg_line.split(',')[1].strip())
            file.write(f"{n_c}, {n_frames}, {output_file}\n")
        else:
            file.write(f"0, 0, {output_file}\n")

        # line 2 : estimated post position with EM probe pivot calibration
        if em_line:
            file.write(f"{em_line}\n")
        else:
            file.write(f"{0.0:12.2f}, {0.0:12.2f}, {0.0:12.2f}\n") # write zeros in place

        # line 3 : estimated post position with optical probe pivot calibration
        if opt_line:
            file.write(f"{opt_line}\n")
        else:
            file.write(f"{0.0:12.2f}, {0.0:12.2f}, {0.0:12.2f}\n") # write zeros in place

        # all lines onward
        for line in reg_rest:
            file.write(line)


#added for P2
def write_em_output(output_file, em_frames):
    n_frames = len(em_frames)

    with open(output_file, 'w') as file:
        # line 1: N_frames, NAME-OUTPUT.TXT
        file.write(f"{n_frames}, {output_file}\n")

        for frame in em_frames:
            for v in frame.v_points:
                file.write(f"{v.x:.2f}, {v.y:.2f}, {v.z:.2f}\n")


