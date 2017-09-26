#ifndef FGRALG_H
#define FGRALG_H

/* --------------------------------------- */

#define USE_OMP

#define USE_ABSOLUTE_SCALE		0		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)

#define DIV_FACTOR				1.4		// Division factor used for graduated non-convexity
#define MAX_CORR_DIST			0.025	// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
#define ITERATION_NUMBER		120		// Maximum number of iteration
#define TUPLE_SCALE				0.90	// Similarity measure used for tuples of feature points.
#define TUPLE_MAX_CNT			1000	// Maximum tuple numbers.
#define NORMALS_SEARCH_RADIUS	0.03	// Normals estimation search radius
#define FPFH_SEARCH_RADIUS		0.2		// FPFH estimation search radius

//#define VERBOSE
#define SHOW_COMPUTATION_TIME

/* --------------------------------------- */

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <flann/flann.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ctime>

using namespace Eigen;
using namespace std;

typedef vector<Vector3f> Points;
typedef vector<VectorXf> Feature;

class FastGlobalRegistration{
public:
	FastGlobalRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_P, pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_Q);
	Eigen::Matrix4f performRegistration();
	Eigen::Matrix4f GetTrans();

private:
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void SearchFLANNTree(flann::Index<flann::L2<float>>* index,
							VectorXf& input,
							std::vector<int>& indices,
							std::vector<float>& dists,
							int nn);
	void AdvancedMatching();
	void NormalizePoints();
	double OptimizePairwise(bool decrease_mu_, int numIter_);

	vector<Points> pointcloud_;
	vector<Feature> features_;
	Matrix4f TransOutput_;
	vector<pair<int, int>> corres_;
	// for normalization
	Points Means;
	float GlobalScale;
	float StartScale;
};

#endif