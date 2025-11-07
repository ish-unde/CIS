import numpy as np
import os

def read_output_file(filename):
    """Read output file and return data as numpy arrays"""
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    
    data_lines = lines[1:] if len(lines) > 1 else []
    
    d_values = []
    c_values = []
    distances = []
    
    for line in data_lines:
        parts = line.split()
        if len(parts) >= 7:
            d_values.append([float(parts[0]), float(parts[1]), float(parts[2])])
            c_values.append([float(parts[3]), float(parts[4]), float(parts[5])])
            distances.append(float(parts[6]))
    
    return np.array(d_values), np.array(c_values), np.array(distances)

def calculate_output_error(your_output_file, debug_answer_file):
    """
    Calculate the error between your output and the debug answer file
    Returns average errors for d_k, c_k, and distances
    """
    
    # read files
    your_d, your_c, your_dist = read_output_file(your_output_file)
    answer_d, answer_c, answer_dist = read_output_file(debug_answer_file)
    
    min_samples = min(len(your_d), len(answer_d))
    if len(your_d) != len(answer_d):
        print(f"Warning: Different number of samples - your: {len(your_d)}, answer: {len(answer_d)}")
        print(f"Using first {min_samples} samples for comparison")
    
    # errors
    d_error = np.linalg.norm(your_d[:min_samples] - answer_d[:min_samples], axis=1)
    c_error = np.linalg.norm(your_c[:min_samples] - answer_c[:min_samples], axis=1)
    dist_error = np.abs(your_dist[:min_samples] - answer_dist[:min_samples])
    
    # Print results to terminal 
    print(f"=== Output Error Analysis ===")
    print(f"Comparing: {your_output_file} vs {debug_answer_file}")
    print(f"Samples compared: {min_samples}")
    print()
    
    print("d_k errors (your d_k vs answer d_k):")
    print(f"  Average: {np.mean(d_error):.6f}")
    print(f"  Max:     {np.max(d_error):.6f}")
    print(f"  Std:     {np.std(d_error):.6f}")
    print()
    
    print("c_k errors (your c_k vs answer c_k):")
    print(f"  Average: {np.mean(c_error):.6f}")
    print(f"  Max:     {np.max(c_error):.6f}")
    print(f"  Std:     {np.std(c_error):.6f}")
    print()
    
    print("Distance errors (your distance vs answer distance):")
    print(f"  Average: {np.mean(dist_error):.6f}")
    print(f"  Max:     {np.max(dist_error):.6f}")
    print(f"  Std:     {np.std(dist_error):.6f}")
    print()
    
    
    return {
        'd_k_errors': d_error,
        'c_k_errors': c_error, 
        'distance_errors': dist_error,
        'avg_d_error': np.mean(d_error),
        'avg_c_error': np.mean(c_error),
        'avg_dist_error': np.mean(dist_error)
    }

def main():
    data_directory = ".."  
    your_output_file = os.path.join(r"C:\Users\ishit\CIS\CIS\PA3\PROGRAMS", "OUTPUT", "PA3-G-Unknown-Output.txt") 
    debug_answer_file = os.path.join(data_directory, "2025 PA345 Student Data", "PA3-G-Unknown-Output.txt")
    print("Looking for files...")
    print(f"Your output: {os.path.abspath(your_output_file)}")
    print(f"Answer file: {os.path.abspath(debug_answer_file)}")
    print()
    
    if os.path.exists(your_output_file) and os.path.exists(debug_answer_file):
        results = calculate_output_error(your_output_file, debug_answer_file)
        
        # Summary
        print("="*50)
        print("SUMMARY:")
        print(f"d_k average error: {results['avg_d_error']:.6f}")
        print(f"c_k average error: {results['avg_c_error']:.6f}")
        print(f"distance average error: {results['avg_dist_error']:.6f}")
        
            
    else:
        print("Error: One or both files not found")
        print(f"Your output exists: {os.path.exists(your_output_file)}")
        print(f"Answer file exists: {os.path.exists(debug_answer_file)}")
        
        # debugging 
        if not os.path.exists(your_output_file):
            output_dir = os.path.dirname(your_output_file)
            if os.path.exists(output_dir):
                print(f"\nFiles in output directory ({output_dir}):")
                for file in os.listdir(output_dir):
                    print(f"  {file}")
        
        if not os.path.exists(debug_answer_file):
            answer_dir = os.path.dirname(debug_answer_file)
            if os.path.exists(answer_dir):
                print(f"\nFiles in answer directory ({answer_dir}):")
                for file in os.listdir(answer_dir):
                    print(f"  {file}")

if __name__ == "__main__":
    main()