# Fast Global Registration

## Introduction

This is an open source C++ implementation based on the technique presented in
the following [paper](https://github.com/IntelVCL/FastGlobalRegistration/blob/master/docs/fast-global-registration.pdf)):

Fast Global Registration  
Qian-Yi Zhou, Jaesik Park, and Vladlen Koltun  
ECCV 2016  

This implementation is largely based on the [original](https://github.com/IntelVCL/FastGlobalRegistration) work from the paper's authors.

## Compilation

FastGlobalRegistration is compiled using [CMake](https://cmake.org/). 

```
> mkdir build
> cd build
> cmake ..
> make
```


## Running FastGlobalRegistration

The FastGlobalRegistration program takes two point clouds in pcd format:

```
> FastGlobalRegistration -p pointcloud_P.pcd -q pointcloud_Q.pcd
```

and prints the estimated rigid transformation matrix.


### Tuning parameters

The relevant parameters are defined in [fast_global_registration.h](inc/fast_global_registration.h) as defines.
```cpp
#define USE_OMP							// Enable the use of OpenMP for normals and feature estimation
#define USE_ABSOLUTE_SCALE		0		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
#define DIV_FACTOR				1.4		// Division factor used for graduated non-convexity
#define MAX_CORR_DIST			0.025	// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
#define ITERATION_NUMBER		120		// Maximum number of iteration
#define TUPLE_SCALE				0.90	// Similarity measure used for tuples of feature points.
#define TUPLE_MAX_CNT			1000	// Maximum tuple numbers.
#define NORMALS_SEARCH_RADIUS	0.03	// Normals estimation search radius
#define FPFH_SEARCH_RADIUS		0.2		// FPFH estimation search radius

#define VERBOSE
#define SHOW_COMPUTATION_TIME
```

We measure distance relative to the diameter_of_model if **USE_ABSOLUTE_SCALE** is set to 0. It is our default setting for synthetic data. For real world data which you know the absolute scale, change **USE_ABSOLUTE_SCALE** to 1 and define **MAX_CORR_DIST** accordingly.

**MAX_CORR_DIST** determines when the optimization will stop. In general, **MAX_CORR_DIST** (USE_ABSOLUTE_SCALE=1) or **MAX_CORR_DIST * diameter_of_model** (USE_ABSOLUTE_SCALE=0) should be set close to the threshold used to determine if a point pair is a match in global space. If you don't know how to set it, start with the default value **0.025**. Decreasing this parameter sometimes results in tighter alignment.

**TUPLE_MAX_CNT** trades off between speed and accuracy. Increasing it will make the optimization slower but the result can be more accurate.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.md) file for details.

