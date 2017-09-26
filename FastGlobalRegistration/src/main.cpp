#include <iostream>
#include "fgr_options.h"
#include "fgr_errors.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "fast_global_registration.h"

int main(int argc, char** argv)
{
	FGROptions options;

    int rt = options.parse(argc, argv);
    switch(rt){
    	case FGROptions::OPTPARSE_HELP:
        	return SUCCESS;
        case FGROptions::OPTPARSE_FAIL:
        	return ERROR_IN_COMMAND_LINE;
    }

    // Load point Clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_P (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_Q (new pcl::PointCloud<pcl::PointXYZ> ());

	if (pcl::io::loadPCDFile (options.ptCloudP_filename, *ptCloud_P) < 0)  {
		std::cerr << "Error loading point cloud " << options.ptCloudP_filename << std::endl << std::endl;
		return ERROR_PTCLOUD_LOAD;
	} else {
		std::cout << "Point cloud " << options.ptCloudP_filename << " successfully loaded, "<< ptCloud_P->width << " pts." << std::endl;
	}
	if (pcl::io::loadPCDFile (options.ptCloudQ_filename, *ptCloud_Q) < 0)  {
		std::cerr << "Error loading point cloud " << options.ptCloudQ_filename << std::endl << std::endl;
		return ERROR_PTCLOUD_LOAD;
	} else {
		std::cout << "Point cloud " << options.ptCloudQ_filename << " successfully loaded, "<< ptCloud_Q->width << " pts." << std::endl;
	}

	// Global Registration
	FastGlobalRegistration fgr(ptCloud_P, ptCloud_Q);
	Eigen::Matrix4f T = fgr.performRegistration();
	std::cout<<"Resulting Transformation matrix: "<<std::endl;
    std::cout<<T<<std::endl<<std::endl;
}