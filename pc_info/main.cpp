#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <vector>
#include <numeric>   
#include <string>

#define ERROR_IN_COMMAND_LINE  1;
#define  SUCCESS 0
#define  ERROR_UNHANDLED_EXCEPTION 2 

#define DEFAULT_K 10

using namespace std;

inline string get_file_extension(const string& filename){
	size_t lastdot = filename.find_last_of(".");
    if (lastdot == string::npos) return "";
    return filename.substr(lastdot, string::npos); 
}

inline bool is_PLYfile(const string& filename){
	string extension = get_file_extension(filename);
	return (extension.compare(".ply"))?false:true;
}

int main (int argc, char** argv){

	int K;
	string cloud_filename;


	// Gettin input from command line
	po::options_description desc("Required arguments");
    desc.add_options()       
      ("help,h", "Print help messages\n")
      ("cloud,i",         po::value<std::string>(&cloud_filename)->required(), 
        "Point cloud filename [*.pcd|ply] (required).\n") 
      ("k",                po::value<int>(&K)->default_value(DEFAULT_K),
        "Size of the neighborhood.\n")
    ;


    po::options_description hidden;
    // no hidden parameters

    po::options_description all_options;
    all_options.add(desc);

    po::positional_options_description p;
    //no positional parameters

    po::variables_map vm;
    try{
	    po::store(po::command_line_parser(argc, argv).
	            options(all_options).
	            positional(p).
	            run(),
	            vm);

	    if(vm.count("help"))
	    {
	        std::cout << desc << '\n';
	        return 1;
	    }

	    po::notify(vm);

	} catch (po::error& e) { 
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cout << desc << '\n';
		return 1; 
	}

	if(K<1){
		std::cerr << "ERROR: k must be greater than 0" << std::endl << std::endl;
		std::cout << desc << '\n';
		return 1; 
	}


	// Reading pt cloud

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if(is_PLYfile(argv[1])){
		if (pcl::io::loadPLYFile (cloud_filename, *cloud) < 0)  {
			cerr << "Error loading point cloud " << cloud_filename << endl << endl;
			return 1;
		}
	} else {
		if (pcl::io::loadPCDFile (cloud_filename, *cloud) < 0)  {
			cerr << "Error loading point cloud " << cloud_filename << endl << endl;
			return 1;
		}
	}
	cout << "Point cloud " << cloud_filename << " successfully loaded, "<< cloud->width << " pts." << endl;

	//build KD-Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud);
	pcl::PointXYZ searchPoint;

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	float average_distance = 0;


	// computing average NN distance
	for (int pt_idx=0; pt_idx<cloud->width; pt_idx++){
		searchPoint = cloud->points[pt_idx];
		kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		float local_average = accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0)/pointNKNSquaredDistance.size(); 
		average_distance += local_average;
	}

	cout << "Average distance of " << K << " points is " << average_distance << endl;

	/*if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
	for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
	          << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
	          << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
	          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;

	}*/




	return 0;

}


