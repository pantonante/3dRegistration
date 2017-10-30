# Experimenter

## Overview
A python script that allows you to run parameters experiments of 3D registration algorithm automatically. Just describe it in a JSON file.

## Features

Run the experiments on hundreds of different combination of parameters and point clouds, compute the relative _rotation error_ and _translation error_ is extremely boring. The idea behind a registration experiment is always the same: 
1. take a set of point cloud pairs
2. run the registration algorithm on a set of possible parameters combination 
3. compute errors
4. plot graphs
Usually the datasets represent a set of increasing value on _noise_ or _outliers_. This process can be easily automized.

The sets of point clouds and of parameters are described in the JSON file, which represents the experiment descriptor. For further information look at the experiment descriptor format below.

After the experiment the script generates a **LaTeX** or **HTML** report file with the error graphs (rotation and translation) for each parameter-experiment as follow:

<p align="center">
<img alt="Error function" src="http://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%2A%7D%0D%0A%26%5Cepsilon_%7B%5Ctext%7Brotation%7D%7D%3D%5ClVert%5Ctext%7Bangle%7D%5Cbig%28R_%7B%5Ctext%7Bground%20truth%7D%7D%5Ccdot%20R_%7B%5Ctext%7Bestimated%7D%7D%5ET%5Cbig%29%5CrVert%5C%5C%0D%0A%26%5Cepsilon_%7B%5Ctext%7Btranslation%7D%7D%3D%5ClVert%20t_%7B%5Ctext%7Bground%20truth%7D%7D-t_%7B%5Ctext%7Bestimated%7D%7D%20%5CrVert%0D%0A%5Cend%7Balign%2A%7D">
</p>


For a complex experiment, I suggested to use the HTML report because it is interactive (thanks to Plotly.js).

The script can be executed in parallel with itself, for example using more terminals or a custom script.

**Summarizing**:

- **run automatically** hundreds of registration experiments compactly described in a JSON file
- computes **rotation error** and **translation error** for each registration experiment
- generates **LaTeX** or **HTML reports** with graphs (including boxplot for execution time)
- can be executed in parallel

## Dependencies

You can install all dependencies by running the following command.
You need a [anaconda](https://www.continuum.io/downloads) or [miniconda](https://conda.io/miniconda.html) to use the environment setting.

```python
conda env create -f environment.yml 
```

then run

```sh
source activate 3dRegistration
```

## Usage

Before running an experiment, you need to create the JSON descriptor file.

### Experiment descriptor

A simple example of a descriptor is the following:
 
```json
{
    "name": "Simple experiment (test)",
    "exe": "../FastGlobalRegistration/build/FastGlobalRegistration",
    "additional_flags": "-c",
    "report_flag":"-j",
    "output": "report.html",
    "dataset_variable": "Noise",
    "dataset": [
        {
            "sigma": "0.000000",
            "P": "../dataset/bunny_noise/bunny_noise0/ptCloud_P.pcd",
            "Q": "../dataset/bunny_noise/bunny_noise0/ptCloud_Q.pcd",
            "T": "../dataset/bunny_noise/bunny_noise0/trans.txt"
        },
        {
            "sigma": "0.005000",
            "P": "../dataset/bunny_noise/bunny_noise0005/ptCloud_P.pcd",
            "Q": "../dataset/bunny_noise/bunny_noise0005/ptCloud_Q.pcd",
            "T": "../dataset/bunny_noise/bunny_noise0005/trans.txt"
        }
    ],
    "parameters": [
        {
            "name":"Tuple Scale",
            "flag":"--tuple-scale",
            "nominal": "0.95",
            "values": ["0.9","0.95"]
        },
        {
            "name": "Graduated non-convexity",
            "flag": "--div-factor",
            "nominal": "1.2",
            "values": []
        }
    ]
}
```

All fields are **mandatory** except from `output` that can be expressed from the command line. They are:

- `name`: the name of the experiment
- `exe`: the registration algorithm executable, it needs to provide some common interface
    - needs to take two point clouds by specifying `-p` and `-q`
    - needs to generate a json report
- `additional_flags`: set some flags common to every experiment (e.g., select the closed form solution)
- `report_flag`: the flag needed to specify the json output from the algorithm
- `output`: path to the output report (the extension selects the format)
- `dataset_variable`: this represent the x-asis in the graphs
- `dataset`: an array of object, each containing
    - the x-axis value for the pair (this value will represent the x-axis on the output graphs)
    - the point clouds pair (P, Q)
    - the ground truth transformation matrix
- `parameters`: an array of object, each representing a value you want to test
    - `name`: parameter name
    - `flag`: the flag needed to activate/specify this parameter
    - `nominal`: the value that should be used when all the other parameters are evaluated
    - `values`: an array containing all the values you want to explore, if empty it will be considered only with respect to its nominal value (i.e. it will always be specified, but no graph will be generated for it)

### Run the experiment

To run the experiment, just run:

```sh
python experimenter.py descriptor_file.json
```

To run more than one experiment in parallel you can use [GNU parallel](http://www.gnu.org/s/parallel})

```sh
parallel python experimenter.py ::: descriptor_1.json descriptor_2.json
```

## License
MIT Copyright (c) Pasquale Antonante