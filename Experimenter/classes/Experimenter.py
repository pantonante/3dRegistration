"""Main class for experimenter"""

import json
import math
import os
import os.path
import random
import shlex
import string
import subprocess
import sys

import numpy as np
from tqdm import tqdm  # progress bar

from .Templater import BoxPlot, Plot, Templater

class Experimenter:
    """
    Class allowing to run parameters experiments of 3D registration
    algorithm automatically. Just describe it in a JSON file.
    How to use:
    ```
    with open("descriptor.json") as data_file:
        desc = json.load(data_file)
    exp = Experimenter(desc)
    doc = exp.run()
    doc.render("output.html", "html") # or latex
    ```
    """

    # random name for the report, make (reasonably) sure that you can run more instances in parallel
    REPORT_FILENAME = 'report_' + ''.join(random.choices(string.ascii_uppercase + string.digits, k=6)) + '.json'

    def __init__(self, descriptor, verbose=False):
        self.descriptor = descriptor
        self.count = self.count_experiments(descriptor)
        if self.count > 0:
            self.mode = "param_experiment" # a set of parameters against a set of dataset
        else:
            self.count = self.count_datasets(descriptor)
            if self.count == 1:
                self.mode = "simple_experiment" # only one dataset against nominal values only
            else:
                self.mode = "dataset_experiment" # a set of dataset against parameter with nominal values only

        if self.mode != "simple_experiment":
            self.pbar = tqdm(total=self.count, unit="experiment")
        else:
            self.pbar = None

    def count_datasets(self, desc):
        """
        Counts the number of datasets specified in the descriptor
        """
        datasets_count = 0
        for dataset in desc['dataset']:
            if len(dataset["P"]) == len(dataset["Q"]) and len(dataset["P"]) == len(dataset["T"]):
                datasets_count = datasets_count + len(dataset["P"])
            else:
                raise ValueError('Dataset size does\'t match.')
        return datasets_count

    def count_experiments(self, desc):
        """
        Counts the number of parameter experiments specified in the descriptor
        """
        parameters = 0
        for par in desc['parameters']:
            parameters = parameters + len(par['values'])
        datasets = self.count_datasets(desc)
        return parameters*datasets

    def dataset_args(self, pcP, pcQ):
        """
        Sets the datasets in the shell command
        """
        return  " -p " + pcP + " -q " + pcQ

    def add_arg(self, par, val):
        """
        Adds a parameter to the command
        """
        return " " + par + " " + val

    def base_cmd(self, desc):
        """
        Base command to execute the algorithm
        """
        flags = ""
        if desc["additional_flags"] != "":
            flags = " " + desc["additional_flags"]
        return desc["exe"] + flags

    def run_cmd(self, cmd):
        """
        Run the shell command (ignoring the stdout and stderr)
        """
        with open(os.devnull, 'w') as devnull:
            subprocess.run(shlex.split(cmd), stdout=devnull, stderr=devnull)
        if self.pbar is not None:
            self.pbar.update()

    def run_simple_experiment(self, desc):
        """
        Runs a single experiment against a dataset with nominal parameter values
        """
        doc = Templater("") # dummy templater
        diameter = float(desc['ptCloud_diameter'])
        ptCloud_P = desc['dataset'][0]['P'][0]
        ptCloud_Q = desc['dataset'][0]['Q'][0]
        T_file = desc['dataset'][0]['T'][0]
        # create the command to run the alg.
        cmd = self.base_cmd(
            desc) + self.dataset_args(ptCloud_P, ptCloud_Q)
        for par in desc["parameters"]:
            cmd = cmd + self.add_arg(par["flag"], par["nominal"])
        cmd = cmd + " -j " + self.REPORT_FILENAME  # set up the report
        self.run_cmd(cmd)  # execute the algorithm
        # Read and analyze output
        try:
            report = self.parse_report(self.REPORT_FILENAME)
        except IOError:
            report = None
            print('"' + cmd + '" did not produce any result')
        except ValueError:
            report = None
            print('"' + cmd + '" returned invalid JSON')
        if report is not None and report['completed'] is True:
            T_gnd = self.read_ground_truth(T_file)
            T_est = np.matrix(report['transformation'])
            # RMSE
            rmse = float(report['RMSE'][-1])
            [rot_err, tra_err] = self.rot_and_trans_error(T_est, T_gnd)
            if np.isnan(rot_err) or np.isnan(tra_err):
                print('"' + cmd + '" returned nan errors')
                raise FloatingPointError('Errors cannot be NaN')
            print('### TRANSFORMATION MATRIX')
            print(T_est)
            print('### ERRORS')
            print(' RMSE:              ' + str(rmse))
            print(' Rotation Error:    ' + str(rot_err) + ' rad')
            print(' Translation Error: ' + str(tra_err / diameter * 100) + ' % of the diameter')
            print('### TIMINGS')
            for ti in report['timing']:
                print(' ' + ti['tag'] + ': '+ ti['time'] + ' sec')
        else:
            print('The algorithm did not completed its execution')
        self.remove_file(self.REPORT_FILENAME)
        return doc

    def run_param_experiments(self, desc):
        """
        Run the parameters experiment
        """
        timings = {}
        doc = Templater(desc["name"])
        diameter = float(desc['ptCloud_diameter'])
        test_parameters = (par for par in desc["parameters"] if len(par['values']) > 0)
        for par in test_parameters: # for each parameter listed in the descriptor file
            plot_rot = Plot(par["name"] + ", rotation error")
            plot_rot.set_axis_label(desc['dataset_variable'], 'Error (rad)')
            plot_tra = Plot(par["name"] + ", translation error")
            plot_tra.set_axis_label(
                desc['dataset_variable'], 'Error (% of diameter)')
            plot_rmse = Plot(par["name"] + ", RMSE")
            plot_rmse.set_axis_label(desc['dataset_variable'], 'RMSE')
            for val in par["values"]: # for each value listed for that parameter 
                values = []
                y_rot = []
                y_tra = []
                y_rmse = []
                for dataset in desc["dataset"]: # for each dataset (X-axis)
                    values.append(float(dataset['value']))
                    rot_err_avg = []
                    tra_err_avg = []
                    rmse_avg = []
                    for ptCloudIdx in range(0, len(dataset["P"])):
                        ptCloud_P = dataset["P"][ptCloudIdx]
                        ptCloud_Q = dataset["Q"][ptCloudIdx]
                        T_file = dataset["T"][ptCloudIdx]
                        cmd = self.base_cmd(desc) + self.dataset_args(ptCloud_P, ptCloud_Q) # create the command to run the alg. on the current dataset
                        cmd = cmd + self.add_arg(par["flag"], val) # add the current parameter with its tested value
                        other_params = ( xpar for xpar in desc["parameters"] if par["flag"] != xpar["flag"] )
                        for xpar in other_params:
                            cmd = cmd + self.add_arg(xpar["flag"], xpar["nominal"])
                        cmd = cmd + " -j " + self.REPORT_FILENAME # set up the report
                        self.run_cmd(cmd) # execute the algorithm
                        # Read and analyze output
                        try:
                            report = self.parse_report(self.REPORT_FILENAME)
                        except IOError:
                            report = None
                            print('"'+cmd+'" did not produce any result')
                        except ValueError:
                            report = None
                            print('"'+cmd+'" returned invalid JSON')

                        if report is not None and report['completed'] is True:
                            T_gnd = self.read_ground_truth(T_file)
                            T_est = np.matrix(report['transformation'])
                            # RMSE
                            rmse = float(report['RMSE'][-1])
                            [rot_err, tra_err] = self.rot_and_trans_error(T_est, T_gnd)
                            if np.isnan(rot_err) or np.isnan(tra_err):
                                print('"'+cmd+'" returned nan errors')
                                raise FloatingPointError('Errors cannot be NaN')
                            rmse_avg.append(rmse)
                            rot_err_avg.append(rot_err)
                            tra_err_avg.append(tra_err / diameter * 100)
                            # Timing
                            for ti in report['timing']:
                                if ti['tag'] in timings:
                                    timings[ti['tag']].append(float(ti['time']))
                                else:
                                    timings[ti['tag']] = [float(ti['time'])]
                    y_rmse.append(np.average(rmse_avg))
                    y_rot.append(np.average(rot_err_avg))
                    y_tra.append(np.average(tra_err_avg))
                # ... new dataset
                plot_rmse.add_datapoints(str(val), values, y_rmse)
                plot_rot.add_datapoints(str(val), values, y_rot)
                plot_tra.add_datapoints(str(val), values, y_tra)
            # .. new value
            doc.add_plot(plot_rmse)
            doc.add_plot(plot_rot)
            doc.add_plot(plot_tra)
        # .. new parameter

        timings_plot = BoxPlot("Timings")
        timings_plot.set_axis_label('Seconds')
        timings_plot.add_datapoints(timings)
        doc.add_plot(timings_plot)

        self.remove_file(self.REPORT_FILENAME)
        return doc

    def run_dataset_experiment(self, desc):
        """
        Run the dataset experiment
        """
        timings = {}
        doc = Templater(desc["name"])
        plot_rot = Plot("Rotation error")
        plot_rot.set_axis_label(desc['dataset_variable'], 'Error (rad)')
        plot_tra = Plot("Translation error")
        plot_tra.set_axis_label(desc['dataset_variable'], 'Error (% of diameter)')
        plot_rmse = Plot("RMSE")
        plot_rmse.set_axis_label(desc['dataset_variable'], 'RMSE')
        diameter = float(desc['ptCloud_diameter'])
        values = []
        y_rot = []
        y_tra = []
        y_rmse = []

        for dataset in desc["dataset"]: # for each dataset (X-axis)
            values.append(float(dataset['value']))
            rmse_avg = []
            rot_err_avg = []
            tra_err_avg = []
            for ptCloudIdx in range(0, len(dataset["P"])):
                ptCloud_P = dataset["P"][ptCloudIdx]
                ptCloud_Q = dataset["Q"][ptCloudIdx]
                T_file = dataset["T"][ptCloudIdx]
                cmd = self.base_cmd(desc) + self.dataset_args(ptCloud_P, ptCloud_Q) # create the command to run the alg. on the current dataset
                for par in desc["parameters"]:
                    cmd = cmd + self.add_arg(par["flag"], par["nominal"])
                cmd = cmd + " -j " + self.REPORT_FILENAME # set up the report
                self.run_cmd(cmd) # execute the algorithm

                # Read and analyze output
                try:
                    report = self.parse_report(self.REPORT_FILENAME)
                except IOError:
                    report = None
                    print('"'+cmd+'" did not produce any result')
                except ValueError:
                    report = None
                    print('"'+cmd+'" returned invalid JSON')

                if report is not None and report['completed'] is True:
                    T_gnd = self.read_ground_truth(T_file)
                    T_est = np.matrix(report['transformation'])
                    # RMSE
                    rmse = float(report['RMSE'][-1])
                    [rot_err, tra_err] = self.rot_and_trans_error(T_est, T_gnd)
                    if np.isnan(rot_err) or np.isnan(tra_err):
                        print('"'+cmd+'" returned nan errors')
                        raise FloatingPointError('Errors cannot be NaN')
                    rmse_avg.append(rmse)
                    rot_err_avg.append(rot_err)
                    tra_err_avg.append(tra_err/diameter*100)
                    # Timing
                    for ti in report['timing']:
                        if ti['tag'] in timings:
                            timings[ti['tag']].append(float(ti['time']))
                        else:
                            timings[ti['tag']] = [float(ti['time'])]
            y_rot.append(np.average(rot_err_avg))
            y_tra.append(np.average(tra_err_avg))
            y_rmse.append(np.average(rmse_avg))
        # ... new dataset

        # Plots
        doc.add_plot(plot_rmse)
        plot_rmse.add_datapoints(desc['name'], values, y_rmse)
        doc.add_plot(plot_rot)
        plot_rot.add_datapoints(desc['name'], values, y_rot)
        doc.add_plot(plot_tra)
        plot_tra.add_datapoints(desc['name'], values, y_tra)
        timings_plot = BoxPlot('Timings')
        timings_plot.set_axis_label('Seconds')
        timings_plot.add_datapoints(timings)
        doc.add_plot(timings_plot)

        self.remove_file(self.REPORT_FILENAME)

        return doc

    def run(self):
        """
        Runs the experiments
        """
        if self.mode == "param_experiment":
            doc = self.run_param_experiments(self.descriptor)
        elif self.mode == "simple_experiment":
            doc = self.run_simple_experiment(self.descriptor)
        elif self.mode == "dataset_experiment":
            doc = self.run_dataset_experiment(self.descriptor)
        return doc

    def parse_report(self, filename):
        """
        Reads the JSON output
        """
        with open(filename) as data_file:
            res = json.load(data_file)
        return res

    def rot_and_trans_error(self, T_est, T_gnd):
        """
        Computes the rotation and translation error
        """
        R_gnd = T_gnd[0:3, 0:3]
        t_gnd = T_gnd[0:3, 3]
        R_est = T_est[0:3, 0:3]
        t_est = T_est[0:3, 3]
        [x, y, z] = self.mat2euler(R_gnd.dot(R_est.transpose()))
        y_rot = np.linalg.norm([x, y, z])
        y_tra = np.linalg.norm(t_est - t_gnd)
        return [y_rot, y_tra]

    def remove_file(self, filename):
        """
        Deletes the report file (if exists)
        """
        try:
            os.remove(filename)
        except OSError:
            pass

    def read_ground_truth(self, filename):
        """
        Parses the Ground Truth matrix
        """
        T_raw = np.loadtxt(filename, delimiter=' ')
        T = np.matrix(T_raw)
        return T

    def mat2euler(self, R):
        """
        Converts rotation matrix to Euler angles
        """
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])
