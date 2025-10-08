CIS PA1 

Name: Ishita Unde and Angela Appiah 

Setup.py contains a brief overview of the packages needed to complete this assignment. 

Run main.py to execute results for each part of the assignment.

Instructions to run main.py: 

1. to run question 4. a, b, c use :  python main.py --calbody_file pa1-debug-a-calbody --calreadings_file pa1-debug-a-calreadings --reg_output_file a_registration
2. to run question 5, use: python main.py --em_file pa1-debug-a-empivot --em_output_file a_em_pivot
3. to run question 6, use: python main.py --opt_file pa1-debug-a-optpivot --opt_output_file a_opt_pivot 
4. to run question 4c, use: python main.py --combine --reg a_registration --em a_em_pivot --opt a_opt_pivot --output a_all_outputs 

All functions are located in math_utils_cart.py with detailed description of functionality. 

testing.py contains all unit tests + synthetic data used to test the functionality of each individual function in math_utils_cart.py. 

file.io contains all functions to open files and parse data.
