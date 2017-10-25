#!/usr/bin/python3

from classes.Templater import * # document generator
from tqdm import * # progress bar
import string, random
import os, os.path
import shlex, subprocess
import numpy as np, math
import sys, time
import argparse
import json

class Experimenter:

	# random name for the report, make (reasonably) sure that you can run more instances in parallel
	REPORT_FILENAME = 'report_' + ''.join(random.choices(string.ascii_uppercase + string.digits, k=6)) + '.json'

	def __init__(self, descriptor, verbose=False):
		self.descriptor = descriptor
		self.pbar = tqdm(total=self.count_experiments(descriptor), unit="experiment")

	def count_experiments(self,desc):
		count = 0
		for par in desc['parameters']:
			for val in par['values']:
				count = count + 1
		count = count * len(desc['dataset'])
		return count

	def dataset_args(self, dataset):
		return  " -p " + dataset["P"] + " -q " + dataset["Q"]

	def add_arg(self, par, val):
		return " " + par + " " + val

	def base_cmd(self, desc):
		flags = ""
		if (desc["additional_flags"]!=""):
			flags = " " + desc["additional_flags"]
		return desc["exe"] + flags

	def run_cmd(self, cmd):
		with open(os.devnull, 'w') as devnull: 
			subprocess.run(shlex.split(cmd), stdout=devnull, stderr=devnull)
		self.pbar.update()

	def run_param_experiments(self, desc):
		timings = {}
		doc = Templater(desc["name"])
		test_parameters = ( par for par in desc["parameters"] if len(par['values'])>0 )
		for par in test_parameters: # for each parameter listed in the descriptor file
			plot_rot=Plot(par["name"]+", rotation error")
			plot_rot.set_axis_label('sigma', 'Error')
			plot_tra=Plot(par["name"]+", translation error")
			plot_tra.set_axis_label('sigma', 'Error')
			for val in par["values"]: # for each value listed for that parameter 
				sigmas = []
				y_rot = []
				y_tra = []
				for dataset in desc["dataset"]: # for each dataset (X-axis)
					sigmas.append(float(dataset['sigma']))
					cmd = self.base_cmd(desc) + self.dataset_args(dataset) # create the command to run the alg. on the current dataset
					cmd = cmd + self.add_arg(par["flag"], val) # add the current parameter with its tested value
					other_params = ( xpar for xpar in desc["parameters"] if par["flag"]!=xpar["flag"] )
					for xpar in other_params:
						cmd = cmd + self.add_arg(xpar["flag"], xpar["nominal"])
					cmd = cmd + " " + desc["report_flag"] + " " + self.REPORT_FILENAME # set up the report
					self.run_cmd(cmd) # execute the algorithm

					# Read and analyze output
					try:
						report = self.parse_report(self.REPORT_FILENAME)
					except IOError:
						print('"'+cmd+'" did not produce any result')
						exit()

					if report['completed'] is True:
						# MSE
						[rot_err, tra_err] = self.rot_and_trans_error(report, dataset["T"])
						y_rot.append(rot_err)
						y_tra.append(tra_err)
						# Timing
						for ti in report['timing']:
							if ti['tag'] in timings:
								timings[ti['tag']].append(float(ti['time']))
							else:
								timings[ti['tag']]=[float(ti['time'])]
				# ... new dataset
				plot_rot.add_datapoints(str(val), sigmas, y_rot)
				plot_tra.add_datapoints(str(val), sigmas, y_tra)
			# .. new value
			doc.add_plot(plot_rot)
			doc.add_plot(plot_tra)
		# .. new parameter

		timings_plot = BoxPlot("Timings")
		timings_plot.set_axis_label('Seconds')
		timings_plot.add_datapoints(timings)
		doc.add_plot(timings_plot)
		self.remove_file(self.REPORT_FILENAME)
		return doc	

	def parse_report(self, filename):
		with open(filename) as data_file:    
		    res = json.load(data_file)
		return res

	def rot_and_trans_error(self, report, ground_truth_filename):
		T_ground = self.read_ground_truth(ground_truth_filename)
		T_gnd = np.matrix(T_ground)
		R_gnd = T_gnd[0:3,0:3]
		t_gnd = T_gnd[0:3,3]
		T = np.matrix(report['transformation'])
		R = T[0:3,0:3]
		t = T[0:3,3]
		[z, y, x] = self.mat2euler(R_gnd.T.dot(R))
		y_rot = np.linalg.norm([x,y,z])
		y_tra = np.linalg.norm(t-t_gnd)
		return [y_rot,y_tra]
		
	def remove_file(self, filename):
		try:
			os.remove(filename)
		except OSError:
			pass

	def read_ground_truth(self, filename):
		T = np.loadtxt(filename, delimiter=' ')
		return T

	def mat2euler(self, M, cy_thresh=None):
	    ''' Euler angle vector from 3x3 matrix

	    Uses the conventions above.

	    Parameters
	    ----------
	    M : array-like, shape (3,3)
	    cy_thresh : None or scalar, optional
	       threshold below which to give up on straightforward arctan for
	       estimating x rotation.  If None (default), estimate from
	       precision of input.

	    Returns
	    -------
	    z : scalar
	    y : scalar
	    x : scalar
	       Rotations in radians around z, y, x axes, respectively

	    Notes
	    -----
	    If there was no numerical error, the routine could be derived using
	    Sympy expression for z then y then x rotation matrix, which is::

	      [                       cos(y)*cos(z),                       -cos(y)*sin(z),         sin(y)],
	      [cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x)],
	      [sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y)]

	    with the obvious derivations for z, y, and x

	       z = atan2(-r12, r11)
	       y = asin(r13)
	       x = atan2(-r23, r33)

	    Problems arise when cos(y) is close to zero, because both of::

	       z = atan2(cos(y)*sin(z), cos(y)*cos(z))
	       x = atan2(cos(y)*sin(x), cos(x)*cos(y))

	    will be close to atan2(0, 0), and highly unstable.

	    The ``cy`` fix for numerical instability below is from: *Graphics
	    Gems IV*, Paul Heckbert (editor), Academic Press, 1994, ISBN:
	    0123361559.  Specifically it comes from EulerAngles.c by Ken
	    Shoemake, and deals with the case where cos(y) is close to zero:

	    See: http://www.graphicsgems.org/

	    The code appears to be licensed (from the website) as "can be used
	    without restrictions".
	    '''
	    M = np.asarray(M)
	    if cy_thresh is None:
	        try:
	            cy_thresh = np.finfo(M.dtype).eps * 4
	        except ValueError:
	            cy_thresh = _FLOAT_EPS_4
	    r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
	    # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
	    cy = math.sqrt(r33*r33 + r23*r23)
	    if cy > cy_thresh: # cos(y) not close to zero, standard form
	        z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
	        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
	        x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
	    else: # cos(y) (close to) zero, so x -> 0.0 (see above)
	        # so r21 -> sin(z), r22 -> cos(z) and
	        z = math.atan2(r21,  r22)
	        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
	        x = 0.0
	    return z, y, x


#############################################################################

def load_descriptor_file(filename):
	with open(filename) as data_file:    
	    desc = json.load(data_file)
	return desc

def main():
	parser = argparse.ArgumentParser(description="Automatic experimenter for 3D registration")
	parser.add_argument("descriptor", help="Filename of the experiment descriptor")
	parser.add_argument("-o", "--output", help="Set output filename (overriding the one in the descriptor file)")
	
	args=parser.parse_args()

	# Loading the experiment descriptor file
	desc=load_descriptor_file(args.descriptor)

	# Infer output type
	if args.output is not None:
		out_filename = args.output
	else:
		out_filename = desc['output']
	extension = os.path.splitext(out_filename)[1][1:]

	if(('tex' or 'latex') in extension):
		file_type = 'latex'
	elif (('html' or 'HTML') in extension):
		file_type = 'html'
	else:
		print('Output extension not supported.')
		exit()

	tic = time.time()
	print("## Evaluating on '" + desc["name"] + "'...")
	# Creating the experimenter class
	exp = Experimenter(desc)
	doc = exp.run_param_experiments(desc)
	# generate the report
	doc.render(out_filename, file_type)
	toc = time.time()
	print('## Finish.')
	print('Experiment completed in ' + str(round(toc-tic,2)) + ' seconds')

##################### MAIN SENTINEL #####################

if __name__ == "__main__":
	main()

