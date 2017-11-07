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
	"""
	Class allowing to run parameters experiments of 3D registration algorithm automatically. Just describe it in a JSON file.
	How to use:
	```
	with open("descriptor.json") as data_file:    
	    desc = json.load(data_file)
	exp = Experimenter(desc)
	doc = exp.run_param_experiments(desc)
	doc.render("output.html", "html") # or latex
	```
	"""

	# random name for the report, make (reasonably) sure that you can run more instances in parallel
	REPORT_FILENAME = 'report_' + ''.join(random.choices(string.ascii_uppercase + string.digits, k=6)) + '.json'

	def __init__(self, descriptor, verbose=False):
		self.descriptor = descriptor
		self.pbar = tqdm(total=self.count_experiments(descriptor), unit="experiment")

	def count_experiments(self,desc):
		parameters = 0
		datasets = 0
		for par in desc['parameters']:
			for val in par['values']:
				parameters = parameters + 1
		for dataset in desc['dataset']:
			if len(dataset["P"])==len(dataset["Q"]) and len(dataset["P"])==len(dataset["T"]):
				datasets = datasets + len(dataset["P"])
			else:
				raise ValueError('Dataset size does\'t match.')
		return parameters*datasets

	def dataset_args(self, P, Q):
		return  " -p " + P + " -q " + Q

	def add_arg(self, par, val):
		return " " + par + " " + val

	def base_cmd(self, desc):
		flags = ""
		if (desc["additional_flags"]!=""):
			flags = " " + desc["additional_flags"]
		return desc["exe"] + flags

	def run_cmd(self, cmd):
		"""
		Run the shell command (discarding the stdout and stderr)
		"""
		with open(os.devnull, 'w') as devnull: 
			subprocess.run(shlex.split(cmd), stdout=devnull, stderr=devnull)
		self.pbar.update()

	def run_param_experiments(self, desc):
		"""
		Run all the experiment 
		"""
		timings = {}
		doc = Templater(desc["name"])
		test_parameters = ( par for par in desc["parameters"] if len(par['values'])>0 )
		for par in test_parameters: # for each parameter listed in the descriptor file
			plot_rot=Plot(par["name"]+", rotation error")
			plot_rot.set_axis_label(desc['dataset_variable'], 'Error (rad)')
			plot_tra=Plot(par["name"]+", translation error")
			plot_tra.set_axis_label(desc['dataset_variable'], 'Error')
			for val in par["values"]: # for each value listed for that parameter 
				values = []
				y_rot = []
				y_tra = []
				for dataset in desc["dataset"]: # for each dataset (X-axis)
					values.append(float(dataset['value']))
					rot_err_avg = []
					tra_err_avg = []
					for ptCloudIdx in range(0, len(dataset["P"])):
						ptCloud_P = dataset["P"][ptCloudIdx]
						ptCloud_Q = dataset["Q"][ptCloudIdx]
						T_file = dataset["T"][ptCloudIdx]
						cmd = self.base_cmd(desc) + self.dataset_args(ptCloud_P, ptCloud_Q) # create the command to run the alg. on the current dataset
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
							T_gnd = self.read_ground_truth(T_file)
							T_est = np.matrix(report['transformation'])
							# MSE
							[rot_err, tra_err] = self.rot_and_trans_error(T_est, T_gnd)
							if np.isnan(rot_err) or np.isnan(tra_err):
								raise FloatingPointError('Errors cannot be NaN')
							rot_err_avg.append(rot_err)
							tra_err_avg.append(tra_err)
							# Timing
							for ti in report['timing']:
								if ti['tag'] in timings:
									timings[ti['tag']].append(float(ti['time']))
								else:
									timings[ti['tag']]=[float(ti['time'])]

					y_rot.append(np.average(rot_err_avg))
					y_tra.append(np.average(tra_err_avg))
				# ... new dataset
				plot_rot.add_datapoints(str(val), values, y_rot)
				plot_tra.add_datapoints(str(val), values, y_tra)
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

	def rot_and_trans_error(self, T_est, T_gnd):
		R_gnd = T_gnd[0:3,0:3]
		t_gnd = T_gnd[0:3,3]
		R_est = T_est[0:3,0:3]
		t_est = T_est[0:3,3]
		[x,y,z] = self.mat2euler(R_gnd.dot(R_est.transpose()))
		y_rot = np.linalg.norm([x,y,z])
		y_tra = np.linalg.norm(t_est-t_gnd)
		return [y_rot,y_tra]
		
	def remove_file(self, filename):
		try:
			os.remove(filename)
		except OSError:
			pass

	def read_ground_truth(self, filename):
		T_raw = np.loadtxt(filename, delimiter=' ')
		T = np.matrix(T_raw)
		return T

	def mat2euler(self, R):
		sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
		singular = sy < 1e-6
		if not singular:
			x = math.atan2(R[2,1] , R[2,2])
			y = math.atan2(-R[2,0], sy)
			z = math.atan2(R[1,0], R[0,0])
		else:
			x = math.atan2(-R[1,2], R[1,1])
			y = math.atan2(-R[2,0], sy)
			z = 0

		return np.array([x, y, z])

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
	print("## Evaluating '" + desc["name"] + "'...")
	# Creating the experimenter class
	exp = Experimenter(desc)
	doc = exp.run_param_experiments(desc)
	# generate the report
	doc.render(out_filename, file_type)
	toc = time.time()
	print('## Finish.')

	print('Experiment completed in ' + str(round(toc-tic,2)) + ' seconds')
	print('Report generated: ' + out_filename)

##################### MAIN SENTINEL #####################

if __name__ == "__main__":
	main()

