/****************************************************************

3D Euclidean Distance Transform Class

Functions computing DT are derived from "dt.c" written by
Alexander Vasilevskiy and Jiaolong Yang.

****************************************************************/

#pragma once

#include <climits>
#include <cstdint>
#include <iostream>

namespace goicp {
namespace distance_transform {

constexpr auto kInfinity = SHRT_MAX;

typedef struct DEucl {
  int v, h;
  float distance;
} DEucl;

typedef struct DEucl3D {
  short v, h, d;
  float distance;
} DEucl3D;

template <class T>
struct Array3d {
  int Xdim, Ydim, Zdim;
  T ***data;
  T *data_array;

  void init(int x, int y, int z);

  // Call this printslice;
  void printArrayDE(int x);
  Array3d() {
    data = nullptr;
    data_array = nullptr;
  }
  ~Array3d() {
    if (data && data[0]) delete (data[0]);
    if (data) delete (data);
    if (data_array) delete (data_array);
  }
};

template <class T>
void Array3d<T>::init(int x, int y, int z) {
  Xdim = x;
  Ydim = y;
  Zdim = z;
  // allocate the memory for the first level pointers.
  int n1 = z, n2 = y, n3 = x;

  data = new T **[n1];

  // set the first level pointers and allocate the memory for the second level pointers.
  {
    data[0] = new T *[n1 * n2];
    for (int row1_index = 0; row1_index < n1; row1_index++)
      data[row1_index] = data[0] + n2 * row1_index;
  }

  T *array_ptr = new T[n1 * n2 * n3];
  data_array = array_ptr;

  // set the second level pointers.
  for (int row1_index = 0; row1_index < n1; row1_index++)
    for (int row2_index = 0; row2_index < n2; row2_index++) {
      data[row1_index][row2_index] = array_ptr;
      array_ptr += n3;
    }
}

template <class T>
void Array3d<T>::printArrayDE(int x) {
  int z, y;
  std::cout << "slice " << x;
  for (y = 0; y < Ydim; y++) {
    std::cout << std::endl;
    for (z = 0; z < Zdim; z++)
      if (data[z][y][x].distance >= kInfinity)
        std::cout << "(*,*,*,*)";
      else
        std::cout << "(" << data[z][y][x].v << "," << data[z][y][x].h << "," << data[z][y][x].d
                  << data[z][y][x].distance << ")";
  }
  std::cout << std::endl;
}

typedef Array3d<DEucl3D> Array3dDEucl3D;
typedef Array3d<float> Array3dfloat;

class DistanceTransform {
 public:
  DistanceTransform();
  int SIZE;
  double scale;
  double expandFactor;
  double xMin, xMax, yMin, yMax, zMin, zMax;
  void build(double *x, double *y, double *z, int num);
  float distance(double x, double y, double z);

 private:
  Array3dDEucl3D A;
};

}  // namespace distance_transform
}  // namespace goicp