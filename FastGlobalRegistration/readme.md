# Fast Global Registration

## Introduction

This is an open source C++ implementation based on the technique presented in the following [paper](https://doi.org/10.1007/978-3-319-46475-6_47)):

Zhou QY., Park J., Koltun V. *Fast Global Registration*. Computer Vision – ECCV 2016, vol 9906

## Dependencies
* [CMake](https://cmake.org/)
* [PCL](http://pointclouds.org) 1.8 (this will install all the other dependencies)


## Compilation

FastGlobalRegistration is compiled using CMake. 

```
> mkdir build
> cd build
> cmake ..
> make
```

Tested and MacOS 10.12.


## Usage

The FastGlobalRegistration program takes two point clouds, P and Q, in pcd format:

```
> FastGlobalRegistration -p pointcloud_P.pcd -q pointcloud_Q.pcd
```

and prints the estimated rigid transformation matrix that aligns Q to P.

To see all available options, execute the program whit the flag `-h` 

```
>./FastGlobalRegistration -h
Required arguments:
  -p [ --pointcloudP ] arg              Point cloud filename [*.pcd].
  -q [ --pointcloudQ ] arg              Point cloud filename [*.pcd].

Miscellaneous:
  -h [ --help ]                         Print help messages
  -v [ --verbose ]                      Verbose output.
  -o [ --output ] arg                   Output filename, save the transformation matrix.
  -f [ --fitness ] arg                  Save to file the RMSE in each iteration.
  -r [ --report ] arg                   Save an HTML report.


Algorithm parameters:
  -a [ --abs-scale ]                    If enabled, measure distance in absolute scale, otherwise in scale relative to the diameter of the model.
  -c [ --closed-form ]                  Use closed form solution for transformation estimation.
  --div-factor arg (=1.4)               Division factor used for graduated non-convexity.
  --max-corr-dist arg (=0.025)          Maximum correspondence distance (also see abs-scale).
  -n [ --iterations ] arg (=64)         Maximum number of iteration.
  --tuple-scale arg (=0.95)             Similarity measure used for tuples of feature points.
  --tuple-max-count arg (=1000)         Maximum tuple numbers.
  --normals-search-radius arg (=0.02)   Normals estimation search radius (see abs-scale).
  --fpfh-search-radius arg (=0.2)       FPFH estimation search radius (see abs-scale).
```

If the `abs-scale` flag is not enabled, all the distances of the model (e.g., search radii and correspondence distance) are measured relatively to the diameter of the point cloud. This is the default behavior with synthetic data. For real-world data, where the absolute scale is known a priori, these parameters can be set accordingly.

The option `max-corr-dist` determines when the optimization will stop. In general, it should be set close to the threshold used to determine if a point pair is a match in global space. If you don't know how to set it, start with the default value **0.025**. Decreasing this parameter sometimes results in tighter alignment.

The option `tuple-max-count` trades off between speed and accuracy. Increasing it will make the optimization slower, but the result can be more accurate.

By enabling the flag `-c` (default disabled), the transformation matrix is estimated by closed form solution. In particular, it's used the Horn’s method for the registration of 3D point clouds.

### Reports
It is possible to export an HTML report of the registration process, it contains:

* Final transformation matrix
* Timing information
* RMSE vs/ iterations
* A summary of the parameters 

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.md) file for details.

