#define SUCCESS 1
#define ERROR_CLI -1
#define ERROR_PTCLOUD_LOAD -2

#define PT_SIZE 2

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h             this help\n"
            << "-p filename    Point Cloud P [*.pcd]\n"
            << "-q filename    Point Cloud Q [*.pcd]\n"
            << "-t filename    Transformation Matrix (optional)\n"
            << "-i             Invert transormation matrix before applying it\n"
            << "\n\n";
}

int main (int argc, char** argv){
  bool transform(false);
  bool inv_T(false);
  std::string ptCloudP_filename, ptCloudQ_filename, trans_filename;
  Eigen::Matrix3f R;
  Eigen::Vector3f t;

  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (argc < 3 || pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return ERROR_CLI;
  }
  if (pcl::console::find_argument (argc, argv, "-p") >= 0){
    pcl::console::parse_argument (argc, argv, "-p", ptCloudP_filename);
  } else {
    printUsage(argv[0]);
    return ERROR_CLI;
  }
  if (pcl::console::find_argument (argc, argv, "-q") >= 0){
    pcl::console::parse_argument (argc, argv, "-q", ptCloudQ_filename);
  } else {
    printUsage(argv[0]);
    return ERROR_CLI;
  }
  if (pcl::console::find_argument (argc, argv, "-t") >= 0){
    transform = true;
    pcl::console::parse_argument (argc, argv, "-t", trans_filename);
  }
  if (pcl::console::find_argument (argc, argv, "-i") >= 0){
    inv_T = true;
  }

  // -------------------------------------
  // ----------Load point clouds----------
  // -------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_P (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (ptCloudP_filename, *ptCloud_P) < 0)  {
    cerr << "Error loading point cloud " << ptCloudP_filename << endl << endl;
    return ERROR_PTCLOUD_LOAD;
  } else {
    cout << "Point cloud " << ptCloudP_filename << " successfully loaded, "<< ptCloud_P->width << " pts.";
    cout << "\tColor: Green" << endl;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud_Q (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (ptCloudQ_filename, *ptCloud_Q) < 0)  {
    cerr << "Error loading point cloud " << ptCloudQ_filename << endl << endl;
    return ERROR_PTCLOUD_LOAD;
  } else {
    cout << "Point cloud " << ptCloudQ_filename << " successfully loaded, "<< ptCloud_P->width << " pts.";
    cout << "\tColor: Red" << endl;
  }

  // -------------------------------------
  // --------Transform pt. cloud Q--------
  // -------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr Q_transformed (new pcl::PointCloud<pcl::PointXYZ> ());
  if(transform){
    cout<< "Transformation matrix "<<trans_filename<<endl;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    ifstream f(trans_filename);
    for (int i = 0; i < 4; i++){
      cout<< "\t";
      for (int j = 0; j < 4; j++){
        f >> T(i, j);
        cout << T(i, j)<<" ";
      }
      cout<<endl;
    }
    f.close();

    if(inv_T){ //if transformation inversion has been requested
      R = T.block<3, 3>(0, 0);
      t = T.block<3, 1>(0, 3);
      T.block<3, 3>(0, 0) = R.transpose();
      T.block<3, 1>(0, 3) = -R.transpose()*t;
      cout << "Inverse matrix: " << endl << T << endl; 
    }

    pcl::transformPointCloud (*ptCloud_Q, *Q_transformed, T);
  }

  // -------------------------------------
  // --------------Visualize--------------
  // -------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_P(ptCloud_P, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_Q(ptCloud_Q, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (ptCloud_P, color_P, "PtCloud_P");
  if(transform)
    viewer->addPointCloud<pcl::PointXYZ> (Q_transformed, color_Q, "PtCloud_Q");
  else
    viewer->addPointCloud<pcl::PointXYZ> (ptCloud_Q, color_Q, "PtCloud_Q");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PT_SIZE, "PtCloud_P");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PT_SIZE, "PtCloud_Q");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return SUCCESS;
}
