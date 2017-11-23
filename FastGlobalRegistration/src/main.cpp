#include <iostream>
#include <vector>
#include "fgr_options.h"
#include "fgr_errors.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include "file_extension.h"
#include "fast_global_registration.h"
#include "report_generator.h"
#include "art.h"

using namespace std;

int main(int argc, char** argv)
{
    // display cool header "3D Registration" (in art.h)
	cout<<CLI_HEADER<<endl<<endl;

	// Instantiate and parse fast global registration options
	FGROptions options;

    int rt = options.parse(argc, argv);
    switch(rt){
    	case FGROptions::OPTPARSE_HELP:
        	return SUCCESS;
        case FGROptions::OPTPARSE_FAIL:
        	return ERROR_IN_COMMAND_LINE;
    }

    // Instantiate and load point Clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_P (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_Q (new pcl::PointCloud<pcl::PointXYZ> ());

	if(is_PLYfile(options.ptCloudP_filename)){
		if (pcl::io::loadPLYFile (options.ptCloudP_filename, *ptCloud_P) < 0)  {
			cerr << "Error loading point cloud " << options.ptCloudP_filename << endl << endl;
			return ERROR_PTCLOUD_LOAD;
		}
	} else {
		if (pcl::io::loadPCDFile (options.ptCloudP_filename, *ptCloud_P) < 0)  {
			cerr << "Error loading point cloud " << options.ptCloudP_filename << endl << endl;
			return ERROR_PTCLOUD_LOAD;
		}
	}
	cout << "Point cloud " << options.ptCloudP_filename << " successfully loaded, "<< ptCloud_P->width << " pts." << endl;

	if(is_PLYfile(options.ptCloudQ_filename)){
		if (pcl::io::loadPLYFile (options.ptCloudQ_filename, *ptCloud_Q) < 0)  {
			cerr << "Error loading point cloud " << options.ptCloudQ_filename << endl << endl;
			return ERROR_PTCLOUD_LOAD;
		}
	} else {
		if (pcl::io::loadPCDFile (options.ptCloudQ_filename, *ptCloud_Q) < 0)  {
			cerr << "Error loading point cloud " << options.ptCloudQ_filename << endl << endl;
			return ERROR_PTCLOUD_LOAD;
		}
	}
	cout << "Point cloud " << options.ptCloudQ_filename << " successfully loaded, "<< ptCloud_Q->width << " pts." << endl;

	if(options.verbose)
		cout<<endl<<"Warning: verbose execution enabled, timing information might be wrong."<<endl<<endl;

	// Instantiate Global Registration engine
	FastGlobalRegistration fgr(ptCloud_P, ptCloud_Q, options);
	

	// Perform Fast Global Registration and display results
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
    
    // Report
    JSONreport(fgr, options.JSONreport);
    HTMLreport(fgr, options.HTMLreport);
}
