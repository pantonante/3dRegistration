# Fast Global Registration

## Introduction

This is an open source C++ implementation based on the technique presented in the following [paper](https://doi.org/10.1007/978-3-319-46475-6_47):

> Zhou QY., Park J., Koltun V. *Fast Global Registration*. Computer Vision – ECCV 2016

## Objective
Consider two point sets P and Q. Our task is to find a **rigid transformation T** that aligns Q to P. The algorithm optimizes a robust objective on correspondences K between P and Q. These correspondences are established by rapid feature matching (**FPFH**) that is performed before the objective is optimized. The correspondences are not recomputed during the optimization. The objective has the following form:

<p align="center">
<img src="http://latex.codecogs.com/svg.latex?E%28%5Cbm%7BT%7D%29%3D%5Csum_%7B%28p%2Cq%29%5Cin%5Cmathcal%7BK%7D%7D%5Crho%28%7B%5Cnorm%7Bq-%5Cbm%7BT%7Dp%7D%7D%29">
</p>

where (p,q) in K are the correspondent points in P and Q. The **robust penalty function**, si the _scaled Geman-McClure estimator_.

<p align="center">
<img src="http://latex.codecogs.com/svg.latex?%5Crho%28x%29%3D%5Cfrac%7B%5Cmu%20x%5E2%7D%7B%5Cmu%2Bx%5E2%7D">
</p>

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

To add the debug symbols (with no code optimization) add the flag run `cmake -DCMAKE_BUILD_TYPE=Debug ..`.

Tested and MacOS 10.12 and Ubuntu 16.04.


## Usage

The FastGlobalRegistration program takes two point clouds, P and Q, in pcd format:

```
> ./FastGlobalRegistration -p pointcloud_P.pcd -q pointcloud_Q.pcd
```

and prints the estimated rigid transformation matrix that aligns Q to P.

To see all available options, execute the program whit the flag `-h` 

```
>./FastGlobalRegistration -h
Required arguments:
  -p [ --pointcloudP ] arg              Point cloud filename [*.pcd].
  -q [ --pointcloudQ ] arg              Point cloud filename [*.pcd].


Miscellaneous:
  -h [ --help ]                         Print help messages.
  
  -v [ --verbose ]                      Verbose output.
  
  -o [ --output ] arg                   Output filename, save the
                                        transformation matrix.
                                        
  -r [ --report ] arg                   Save an HTML report.
  
  -j [ --json ] arg                     Save the report as JSON file.

Algorithm parameters:
  -a [ --abs-scale ]                    If enabled, measure distance in
                                        absolute scale, otherwise in scale
                                        relative to the diameter of the model.

  -c [ --closed-form ]                  Use closed form solution for
                                        transformation estimation.

  --div-factor arg (=1.4)               Division factor used for graduated
                                        non-convexity.

  --stop-rmse arg (=0.01)               Optimization stops when reach the given
                                        RMSE.

  --max-corr-dist arg (=0.025)          Maximum correspondence distance (also
                                        see abs-scale).

  -n [ --iterations ] arg (=64)         Maximum number of iteration.

  --tuple-scale arg (=0.95)             Similarity measure used for tuples of
                                        feature points.

  -m [ --tuple-max-count ] arg (=100)   Maximum tuple numbers.

  --normals-search-radius arg (=0.03)   Normals estimation search radius.

  --fpfh-search-radius arg (=0.2)       FPFH estimation search radius.
```

If the `abs-scale` flag is not enabled, all the distances of the model (e.g., search radii and correspondence distance) are measured relatively to the diameter of the point cloud. This is the default behavior with synthetic data. For real-world data, where the absolute scale is known a priori, these parameters can be set accordingly.

The options `stop-rmse` and `max-corr-dist` determine when the optimization will stop. The first one is specifies the desired RMSE, the latter specifies a stoppng criteria on the graduated non-convexity parameter. In general, it should be set close to the threshold used to determine if a point pair is a match in global space. If you don't know how to set it, start with the default value _0.025_. Decreasing this parameter sometimes results in tighter alignment.

The option `tuple-max-count` trades off between speed and accuracy. Increasing it will make the optimization slower, but the result can be more accurate.

By enabling the flag `-c` (default disabled), the transformation matrix is estimated by closed form solution. In particular, it's used the Horn’s method for the registration of 3D point clouds.

### Reports
It is possible to export a JSON od HTML report of the registration process, it contains:

* Final transformation matrix
* Timing information
* RMSE vs/ iterations
* A summary of the parameters

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.md) file for details.

