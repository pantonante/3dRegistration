#include <iostream>
#include <vector>
#include "fgr_options.h"
#include "fgr_errors.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "fast_global_registration.h"
#include "report_generator.h"
#include "art.h"

using namespace std;

int main(int argc, char** argv)
{
	cout<<CLI_HEADER<<endl<<endl;

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
		cerr << "Error loading point cloud " << options.ptCloudP_filename << endl << endl;
		return ERROR_PTCLOUD_LOAD;
	} else {
		cout << "Point cloud " << options.ptCloudP_filename << " successfully loaded, "<< ptCloud_P->width << " pts." << endl;
	}
	if (pcl::io::loadPCDFile (options.ptCloudQ_filename, *ptCloud_Q) < 0)  {
		cerr << "Error loading point cloud " << options.ptCloudQ_filename << endl << endl;
		return ERROR_PTCLOUD_LOAD;
	} else {
		cout << "Point cloud " << options.ptCloudQ_filename << " successfully loaded, "<< ptCloud_Q->width << " pts." << endl;
	}

	if(options.verbose)
		cout<<endl<<"Warning: verbose execution enabled, timing information might be wrong."<<endl<<endl;

	// Global Registration
	FastGlobalRegistration fgr(ptCloud_P, ptCloud_Q, options.verbose);
	fgr.closed_form				= options.closed_form;
	fgr.use_absolute_scale 		= options.use_absolute_scale;		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
	fgr.div_factor 				= options.div_factor; 				// Division factor used for graduated non-convexity
	fgr.max_corr_dist 			= options.max_corr_dist;			// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
	fgr.iteration_number 		= options.iteration_number;			// Maximum number of iteration
	fgr.tuple_scale 			= options.tuple_scale;				// Similarity measure used for tuples of feature points.
	fgr.tuple_max_count 		= options.tuple_max_count;			// Maximum tuple numbers.
	fgr.stop_mse				= options.stop_mse;					// Stop criteria
	fgr.normals_search_radius 	= options.normals_search_radius;	// Normals estimation search radius
	fgr.fpfh_search_radius 		= options.fpfh_search_radius;		// FPFH estimation search radius
	Eigen::Matrix4f T = fgr.performRegistration();

	cout<<"--------------------------------------------------"<<endl;
	cout<<"Transformation matrix: "<<endl;
    cout<<T<<endl;
    cout<<"--------------------------------------------------"<<endl;
	cout<<"RMSE: "<< fgr.getRMSE()<<endl;
	cout<<"--------------------------------------------------"<<endl;
	cout<<fgr.getTiming()<<endl;
	cout<<"--------------------------------------------------"<<endl<<endl;

    // Save transformation matrix
    if(options.outputfile.compare("")){
    	cout<<"Saving transformation matrix to file..."<<endl;
    	ofstream fid;
		fid.open (options.outputfile);
		fid << T;
		fid.close();
    }
    //save fitness value over iterations
    if(options.fitnessfile.compare("")){
    	cout<<"Saving RMSE to file..."<<endl;
    	ofstream fid;
		fid.open (options.fitnessfile);
    	for (int i = 0; i < fgr.fitness.size(); i++)
    		fid << i<<","<< sqrt(fgr.fitness[i]) << endl;
		fid.close();
    }

    // Report generation
    generateReport(fgr, options.reportfile);
}