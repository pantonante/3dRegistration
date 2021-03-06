#ifndef FGRALG_H
#define FGRALG_H

#include <vector>
#include <numeric>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <flann/flann.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "CPUTimer.h"
#include "default_config.h"
#include "fgr_options.h"

using namespace Eigen;
using namespace std;

typedef vector<Vector3f> Points;
typedef vector<VectorXf> Feature;

class FastGlobalRegistration{
public:
	bool	closed_form 			= CLOSED_FORM;
	bool 	verbose 				= VERBOSE;
	bool 	use_absolute_scale 		= USE_ABSOLUTE_SCALE;	// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
	float 	div_factor 				= DIV_FACTOR; 			// Division factor used for graduated non-convexity
	int 	iteration_number 		= ITERATION_NUMBER;		// Maximum number of iteration
	float 	tuple_scale 			= TUPLE_SCALE;			// Similarity measure used for tuples of feature points.
	int 	tuple_max_count 		= TUPLE_MAX_CNT;		// Maximum tuple numbers.
	float 	stop_mse				= STOP_RMSE*STOP_RMSE;	// Stop criteria
	float 	normals_search_radius 	= NORMALS_SEARCH_RAD;	// Normals estimation search radius
	float 	fpfh_search_radius 		= FPFH_SEARCH_RAD;		// FPFH estimation search radius

	vector<float> fitness; 									// MSE over iterations

	FastGlobalRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_P, pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_Q, FGROptions options);
	Eigen::Matrix4f performRegistration();
	Eigen::Matrix4f GetTrans();

	inline float getRMSE(){ 
		if (fitness.size()>0) 
			return sqrt(fitness[fitness.size()-1]);
		else
			return -1;
	}
	inline string getTiming(){return timer_.allTimings(); }
	inline vector<TimingInfo> getTimingInfo(){return timer_.getMeasurements();}
	inline int getNumCorrespondences(){return corres_.size();}

private:
	CPUTimer timer_;
	vector<Points> pointcloud_;
	vector<Feature> features_;
	Matrix4f TransOutput_;
	vector<pair<int, int>> corres_;
	// for normalization
	Points Means;
	float GlobalScale;
	float StartScale;

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void SearchFLANNTree(flann::Index<flann::L2<float>>* index,
							VectorXf& input,
							std::vector<int>& indices,
							std::vector<float>& dists,
							int nn);
	void AdvancedMatching();
	void NormalizePoints();
	double OptimizePairwise(int numIter_);
	double OptimizePairwise_ClosedForm(int numIter_);
};

#endif