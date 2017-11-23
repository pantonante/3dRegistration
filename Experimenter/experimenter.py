"""Main for experimenter"""

#!/usr/bin/python3
import argparse
import json
import time

from classes.Experimenter import Experimenter
from classes.Templater import *  # document generator

def load_descriptor_file(filename):
    with open(filename) as data_file:
        desc = json.load(data_file)
    return desc

def main():
    parser = argparse.ArgumentParser(description="Automatic experimenter for 3D registration")
    parser.add_argument("descriptor", help="Filename of the experiment descriptor")
    parser.add_argument("-o", "--output", help="Set output filename (overriding the one in the descriptor file)")
    args = parser.parse_args()

    # Loading the experiment descriptor file
    desc = load_descriptor_file(args.descriptor)

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
    doc = exp.run()
    # generate the report
    rendered = doc.render(out_filename, file_type)
    toc = time.time()
    print('## Finish.')

    print('Experiment completed in ' + str(round(toc-tic,2)) + ' seconds')
    if rendered:
        print('Report generated: ' + out_filename)

##################### MAIN SENTINEL #####################

if __name__ == "__main__":
    main()
