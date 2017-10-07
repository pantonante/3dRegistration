# Visualizer

## Introdiction
Viz is a is a small visualization application built on top of PCL Visualizer to allow the visualization of the result of a 3D registration process.

## Dependencies
* [CMake](https://cmake.org/)
* [PCL](http://pointclouds.org) 1.8 (this will install all the other dependencies)

## Usage

The application expects as input two point cloud in pcd format and an optional transformation matrix (4x4). It shows both point clouds, eventually applying the specified transformation to point cloud Q.

Calling the application with the flag `-h` it shows the help message.

```
Usage: ./viz [options]

Options:
-------------------------------------------
-h             this help
-p filename    Point Cloud P [*.pcd]
-q filename    Point Cloud Q [*.pcd]
-t filename    Transformation Matrix (optional)
```

A typical usage is

```
>./viz -p ptCloud_P.pcd -q ptCloud_Q.pcd -t trans.txt
```

The transformation matrix file contains a 4x4 matrix in ASCII format, each value separated by a space and each row in a new line. For example:

```
-6.415777e-01 7.544783e-01 1.383496e-01 3.293619e-01
-7.223734e-01 -5.336328e-01 -4.397870e-01 2.141375e-01
-2.579819e-01 -3.820976e-01 8.873820e-01 -8.409540e-02
0 0 0 1

```

## License
MIT Copyright (c) 2017 Pasquale Antonante