/* Tutaj odbywa się łączenie automatyczne przy pomocy ICP
 */

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <pcl/point_cloud.h>
#include <iomanip>      // std::setw
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <string.h>
#include <stdlib.h>
 #include <pcl/visualization/cloud_viewer.h>


#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/gicp.h>   //Generalized Iterative Closest Point
#include <pcl/registration/joint_icp.h>   //Joint Iterative Closest Point
#include <pcl/registration/icp_nl.h>   //Iterative Closest Point Non Linear

using namespace std;
using namespace octomap;
using namespace pcl;
typedef pcl::PointXYZINormal PointT;   //Iterative Closest Point With Normals
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::IterativeClosestPointWithNormals<PointT, PointT> icp_normal;

typedef pcl::PointXYZINormal PointXYZ;
typedef pcl::IterativeClosestPoint<PointT,PointT> icp_default;
typedef pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp_general;
typedef pcl::GeneralizedIterativeClosestPoint<PointT,PointT> icp_generalT;
//typedef pcl::IterativeClosestPointWithNormals<pcl::PointXYZ,pcl::PointXYZ> icp_normal;

//global variables
long int timeCnt;
bool next_iteration = false;


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym () == "space" &&event.keyDown ())
   {
      next_iteration = true;
      cout<<"kliknieto spacje"<<endl;
  }
}
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}



// This is the main function
int main (int argc, char** argv)
{


  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_23 (new pcl::PointCloud<pcl::PointXYZ>); // PTR
  pcl::PCLPointCloud2 *cloud2_23 = new pcl::PCLPointCloud2;

 //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_24 (new pcl::PointCloud<pcl::PointXYZ>); // PTR
  pcl::PCLPointCloud2 *cloud2_24 = new pcl::PCLPointCloud2;

 // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr (new pcl::PointCloud<pcl::PointXYZ>); // PTR

  PointCloudT::Ptr cloud_23T (new PointCloudT); // PTR

  PointCloudT::Ptr cloud_24T (new PointCloudT); // PTR

  PointCloudT::Ptr cloud_trT (new PointCloudT); // PTR



  string infilename23 = "/home/szymon/Pulpit/Inż/Zdjęcia/Trasa 4/cloud00023.pcd";
  string infilename24 = "nowy_24.pcd";

  pcl::console::TicToc time; // nie do końca ogarniam co on robi :/
  time.tic ();               // taka wymuszona pauza ??

     if(pcl::io::loadPCDFile(infilename23, *cloud2_23) == -1) //load the file
    {
      std::cout << "Error loading point cloud 23" << std::endl << std::endl;
      return -1;
    }
     if(pcl::io::loadPCDFile(infilename24, *cloud2_24) == -1) //load the file
    {
      std::cout << "Error loading point cloud 24" << std::endl << std::endl;
      return -1;
    }

   // pcl::fromPCLPointCloud2 (*cloud2_23,*cloud_23);
    //pcl::fromPCLPointCloud2 (*cloud2_24,*cloud_24);

   pcl::fromPCLPointCloud2 (*cloud2_23,*cloud_23T);
   pcl::fromPCLPointCloud2 (*cloud2_24,*cloud_24T);

  int iterations = 10;

  // Executing the transformation*cloud_tr = *cloud_in_2;  // We backup cloud_icp into cloud_tr for later use
  //*cloud_tr = *cloud_24;  // We backup cloud_icp into cloud_tr for later use
  *cloud_trT = *cloud_24T;  // We backup cloud_icp into cloud_tr for later use


  // The Iterative Closest Point algorithm
  time.tic ();


    icp_default* ptr;
   // ptr = new icp_generalT(); // korzystamy gicp narazie

    ptr = new icp_generalT();

    ptr->setMaximumIterations (iterations);
    ptr->setRANSACOutlierRejectionThreshold(0.5);//zasieg szukania ?
    ptr->setInputSource (cloud_24T);
    ptr->setInputTarget (cloud_23T);
    ptr->align (*cloud_24T);
    ptr->setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    timeCnt = time.toc ();

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();


    std::cout << "----------------------------------------------------" << std::endl;
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << timeCnt << " ms" << std::endl;
    //cout precosion
    const int precision = 10;
    if (ptr->hasConverged ())
    {
      std::cout.precision(precision);
      std::cout << "ICP has converged, score is " << ptr->getFitnessScore () << std::endl;
      std::cout << "ICP transformation " << iterations << " : cloud_24 -> cloud_23" << std::endl;
      std::cout << "----------------------------------------------------" << std::endl;
      transformation_matrix = ptr->getFinalTransformation ().cast<double>();
      print4x4Matrix (transformation_matrix);
    }
    else
    {
      PCL_ERROR ("\nICP has not converged.\n");
      return (-1);
    }





  // Visualization

  pcl::visualization::PCLVisualizer viewer1 ("ICP laczenie");

  // Create two verticaly separated viewports
  int v1(0);
  int v2(1);
  //viewer1.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  //viewer1.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;


 // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1 (cloud_23, 0, 0, 255);
 // viewer1.addPointCloud (cloud_23, source_cloud_color_handler1, "v1");
 // viewer1.addPointCloud (cloud_23, source_cloud_color_handler1, "v2");

 // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (cloud_24, 255, 0, 0);
//  viewer1.addPointCloud (cloud_24, source_cloud_color_handler2, "v3");

 // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (cloud_tr, 0, 255, 0);
 // viewer1.addPointCloud (cloud_tr, source_cloud_color_handler3, "v4");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler1 (cloud_23T, 0, 0, 255);
  viewer1.addPointCloud (cloud_23T, source_cloud_color_handler1, "v1");
  viewer1.addPointCloud (cloud_23T, source_cloud_color_handler1, "v2");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler2 (cloud_24T, 255, 0, 0);
  viewer1.addPointCloud (cloud_24T, source_cloud_color_handler2, "v3");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler3 (cloud_trT, 0, 255, 0);
  viewer1.addPointCloud (cloud_trT, source_cloud_color_handler3, "v4");



  viewer1.addCoordinateSystem (1.0, "cloud", 0);
  viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

  cout<<"Chmura 23 jest niebieska\nChmura 24 jest zielona\nChmura ta z ICP jest czerwona\n";


  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  cout<<iterations_cnt<<endl;


  // Register keyboard callback :
  viewer1.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
  viewer1.setSize (1280, 1024);  // Visualiser window size


  while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer1.spinOnce ();
      // The user pressed "space" :
      if (next_iteration)
      {

        // The Iterative Closest Point algorithm
        time.tic ();
        ptr->align (*cloud_24T);
        time.toc ();
        timeCnt += time.toc();

        std::cout << "ANOTHER ITERATION:" << std::endl;
        std::cout << "----------------------------------------------------" << std::endl;
        std::cout << "Applied 1 ICP iterations in " << time.toc() << " ms" << std::endl;
        std::cout << "Applied " << iterations << " ICP iterations in " << timeCnt << " ms" << std::endl;


        int precision =10;
        if (ptr->hasConverged ())
        {
          std::cout.precision(precision);
          std::cout << "ICP has converged, score is " << ptr->getFitnessScore () << std::endl;
          std::cout << "----------------------------------------------------" << std::endl;
          std::cout << "ICP transformation " << ++iterations << " : cloud_in_1 -> cloud_in_2" << std::endl;
          //transformation_matrix *= ptr->getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
          //print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

          //ss.str ("");
          //ss << iterations;
          //std::string iterations_cnt = "ICP iterations = " + ss.str ();

          viewer1.updatePointCloud (cloud_24T, source_cloud_color_handler2, "v3");
          //viewer1.removePointCloud(cloud_24,"v3");
        }
        else
        {
          PCL_ERROR ("\nICP has not converged.\n");
          return (-1);
        }
      }
      next_iteration = false;
    }

    delete ptr;
    return (0);
}
