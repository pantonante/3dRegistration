# Experimenter

## Overview
A python script that allows you to run parameters experiments of 3D registration algorithm automatically. Just describe them in a JSON file.

## Features

Running experiments on hundreds of different combination of parameters and point clouds, compute the relative _rotation error_ and _translation error_ is extremely boring. The idea behind a registration experiment is always the same:
 
1. take a set of point cloud pairs
2. run the registration algorithm on a set of possible parameters combination 
3. compute errors against ground truth
4. plot graphs

Usually, the datasets represent a set of increasing value on _noise_ or _outliers_. This process can be easily automized.

The sets of point clouds and of parameters are described in the JSON file, which represents the experiment descriptor. For further information look at the experiment descriptor format below.

After the experiment the script generates a **LaTeX** or **HTML** report file with the error graphs (rotation and translation) for each parameter-experiment as follow:

<p align="center">
<img alt="Error function" src="http://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%2A%7D%0D%0A%26E_%7B%5Ctext%7Brotation%7D%7D%3D%5ClVert%5Ctext%7Bangle%7D%5Cbig%28R_%7B%5Ctext%7Bground%20truth%7D%7D%5Ccdot%20R_%7B%5Ctext%7Bestimated%7D%7D%5ET%5Cbig%29%5CrVert%5C%5C%0D%0A%26E_%7B%5Ctext%7Btranslation%7D%7D%3D%5ClVert%20t_%7B%5Ctext%7Bground%20truth%7D%7D-t_%7B%5Ctext%7Bestimated%7D%7D%20%5CrVert%0D%0A%5Cend%7Balign%2A%7D">
</p>


For a complex experiment, I suggested using the HTML report because it is interactive (thanks to [Plotly.js](https://plot.ly/javascript/)).

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

## Experiment descriptor

Before running an experiment, you need to create the JSON descriptor file. A simple example of a descriptor is the following:
 
```json
{
    "name": "Parameters experiment",
    "exe": "../build/FastGlobalRegistration",
    "additional_flags": "-c",
    "output": "report.html",
    "ptCloud_diameter": "0.276120",
    "dataset_variable": "Noise",
    "dataset": [
      {
        "value": "0.0",
        "P": [
          "../dataset/bunny_noise/bunny_noise0/ptCloud_P1.pcd",
          "../dataset/bunny_noise/bunny_noise0/ptCloud_P2.pcd"
        ],
        "Q": [
          "../dataset/bunny_noise/bunny_noise0/ptCloud_Q1.pcd",
          "../dataset/bunny_noise/bunny_noise0/ptCloud_Q2.pcd"
        ],
        "T": [
          "../dataset/bunny_noise/bunny_noise0/T1.txt",
          "../dataset/bunny_noise/bunny_noise0/T2.txt"
        ]
      },
      {
          "value": "0.005",
          "P": [
            "../dataset/bunny_noise/bunny_noise0005/ptCloud_P1.pcd",
            "../dataset/bunny_noise/bunny_noise0005/ptCloud_P2.pcd"
          ],
          "Q": [
            "../dataset/bunny_noise/bunny_noise0005/ptCloud_Q1.pcd",
            "../dataset/bunny_noise/bunny_noise0005/ptCloud_Q2.pcd"
          ],
          "T": [
            "../dataset/bunny_noise/bunny_noise0005/T1.txt",
            "../dataset/bunny_noise/bunny_noise0005/T2.txt"
          ]     
      }
    ],
    "parameters": [
      {
        "name": "Tuple Scale",
        "flag": "--tuple-scale",
        "nominal": "0.95",
        "values": ["0.9", "0.95"]
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

All fields are **mandatory** except `output` that can be expressed from the command line. They are:

- `name`: the name of the experiment
- `exe`: the registration algorithm executable, it needs to provide some common interface
    - needs to take two point clouds by specifying `-p` and `-q`
    - needs to generate a supported json report
- `additional_flags`: set some flags common to every experiment (e.g., select the closed form solution)
- `output`: path to the output report (the extension selects the format)
- `dataset_variable`:  represents the x-axis in the graphs
- `ptCloud_diameter`: the diameter of the reference point cloud (pt. Cloud Q); it is useful to compute the translation error as percentage of the diameter
- `dataset`: an array of object, each containing
    - the x-axis value for the pair (this value will represent the x-axis on the output graphs)
    - an array of point clouds P
    - an array of point clouds Q
    - the ground truth transformation matrix for each pair (P, Q)
- `parameters`: an array of object, each representing a value you want to test
    - `name`: parameter name
    - `flag`: the flag needed to activate/specify this parameter
    - `nominal`: the value that should be used when all the other parameters are evaluated
    - `values`: an array containing all the values you want to explore, if empty it will be considered only involving its nominal value (i.e. it will always be specified, but no graph will be generated for it)

Note that the array of point clouds P, Q and the relative T for each x-axis value are homogenous in size since they represent a correspondence.


## Operational modes

There are three operational modes, automatically selected on the data available in the experiment descriptor.

### Simple experiment

If you specify **only a point cloud pair** and **only nominal values for parameters**, you are running a simple experiment. In other words, the algorithm will be executed once on the pair and the output compared against the ground truth.

Descriptor example 

```json
{
    "name": "Simple Experiment",
    "exe": "../build/FastGlobalRegistration",
    "additional_flags": "-a -c",
    "output": "simple_exp.html",
    "dataset_variable": "noise",
    "ptCloud_diameter": "0.232839",
    "dataset": [
      {
        "value": "0.002",
        "P": ["../bunny_noise/bunny0002/ptCloud_P1.pcd"],
        "Q": ["../bunny_noise/ptCloud_Q.pcd"],
        "T": ["../bunny_noise/bunny0002/T1.txt"]
      }
    ],
    "parameters": [
      {
        "name":"Tuple Scale",
        "flag":"--tuple-scale",
        "nominal": "0.95",
        "values": []
      },
      {
        "name": "Graduated non-convexity",
        "flag": "--div-factor",
        "nominal": "1.4",
        "values": []
      }
    ]
}
```

In this case, the output field will be ignored since the report will be printed on the console:

```
## Evaluating 'Simple Experiment'...
### TRANSFORMATION MATRIX
[[-0.202523 -0.514556  0.833201  0.473546]
 [-0.885021 -0.26805  -0.380659  0.731453]
 [ 0.419209 -0.814487 -0.401103  0.521201]
 [ 0.        0.        0.        1.      ]]
### ERRORS
 RMSE:              0.0239006
 Rotation Error:    0.0241262316243 rad
 Translation Error: 0.9564957751 % of the diameter
### TIMINGS
 Features computation: 0.291498 sec
 Matching: 0.090683 sec
 Normalization: 0.006959 sec
 Registration: 0.440064 sec
## Finish.
Experiment completed in 1.55 seconds
```

### Dataset experiment

If you specify **more than one point cloud pair** and **only nominal values for parameter** than you are running a dataset experiment. The algorithm will be executed on all the datasets, eventually averaging on point clouds corresponding to the same value level (note that for each dataset, P, Q, and T are vectors). 

### Parameters experiment

If you specify **more than one point cloud pair** and **values for at least one parameter** than you are running a parameters experiment (see the first example). The algorithm will be executed on all the datasets, eventually averaging on point clouds corresponding on the same value level (note that for each dataset, P, Q, and T are vectors) testing all the parameter combinations.

The parameters combination are computed in the following way: for each parameter, the algorithm is executed using every possible value from the values array, setting all the other parameters with their nominal values.


## Run the experiment

To run the experiment, just run:

```sh
python experimenter.py descriptor_file.json
```

To run more than one experiment in parallel, you can use [GNU parallel](https://www.gnu.org/software/parallel/)

```sh
parallel python experimenter.py ::: descriptor_1.json descriptor_2.json
```

or, if you have a list of descriptor (one descriptor file each line) in a text file called `desc.lst` than

```sh
cat desc.lst | parallel python experimenter.py
```


## License
MIT Copyright (c) Pasquale Antonante