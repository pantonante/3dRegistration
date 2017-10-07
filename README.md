## Overview
Some algorithms and tools for 3D rigid registration.

## Structure
* [Presentation](Presentation) theoretical prime about 3D registration, including the brief analysis of some algorithm. The folder contains the PDF and LateX (Beamer) source.
* [Fast Global Registration](FastGlobalRegistration) C++ implementation of the relative algorithm
* [Point Cloud Generator](PointCloudGenerator) a Matlab implementation of a point cloud generator. Can generate a point cloud from a function or a pre-scanned object.
* [Visualizer](Visualizer) Point Cloud Visualizer, allows to show two point clouds, and optionally apply a rigid transformation to one of them.
* [dataset](dataset) some point clouds ready to be used by the tools
* `evaluate.sh` batch script to run the evaluation on all the available dataset in the relative folder

## License
MIT - see each subfolder for further details