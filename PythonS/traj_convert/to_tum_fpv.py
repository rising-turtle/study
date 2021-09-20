import csv
import numpy as np
import sys
import argparse


input_fname = sys.argv[1]
input_data  = np.loadtxt(input_fname)
output_data  = np.delete(input_data, 0, axis=1); 


insertIndex = len(input_fname)-4;
output_fname = input_fname[:insertIndex] + '_tum' + input_fname[insertIndex:]

np.savetxt(output_fname, output_data, delimiter=' ', fmt=('%10.9f', '%2.6f', '%2.6f', '%2.6f', '%2.6f', '%2.6f', '%2.6f', '%2.6f'))

