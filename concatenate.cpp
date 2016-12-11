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

using namespace std;
using namespace octomap;

int main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 *cloud_color = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 *cloud_basic = new pcl::PCLPointCloud2;


    string infilename23 = "/home/szymon/Pulpit/Inż/Zdjęcia/trasa3/cloud00007.pcd";
    string infilename24 = "/home/szymon/Pulpit/Inż/Zdjęcia/trasa3/cloud00008.pcd";

     // string infilename24 = "/home/szymon/Pulpit/Inż/Zdjęcia/Trasa 4/cloud00024.pcd";

\

       if(pcl::io::loadPCDFile(infilename23, *cloud_basic) == -1) //load the file
      {
        std::cout << "Error loading point cloud 23" << std::endl << std::endl;
        return -1;
      }
       if(pcl::io::loadPCDFile(infilename24, *cloud_color) == -1) //load the file
      {
        std::cout << "Error loading point cloud 24" << std::endl << std::endl;
        return -1;
      }

      pcl::fromPCLPointCloud2 (*cloud_basic,*basic_cloud_ptr);
      pcl::fromPCLPointCloud2 (*cloud_color,*point_cloud_ptr);

      // Visualization
      printf(  "\nPoint cloud colors :  white  = original point cloud\n"
          "                        red  = transformed point cloud\n");
     // pcl::visualization::PCLVisualizer viewer1 ("Matrix transformation example");
     // pcl::visualization::PCLVisualizer viewer2 ("Matrix transformation example");


       // Define R,G,B colors for the point cloud
      //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255.0, 255.0, 255.0);
      // We add the point cloud to the viewer and pass the color handler
      //viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");


     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1 (cloud_23, 0, 0, 255);
     // viewer1.addPointCloud (cloud_23, source_cloud_color_handler1, "original_cloud1");

    //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
 //     viewer1.addPointCloud (basic_cloud_ptr, rgb, "original_cloud2");

      //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
      //viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (cloud_24, 255, 0, 0);
    //  viewer2.addPointCloud (cloud_24, source_cloud_color_handler3, "original_cloud2");

    //  viewer1.addCoordinateSystem (1.0, "cloud", 0);
     // viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    ////  viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");



  ///ZAPISYWANIE DO PCD

          string olp1 = "clou.pcd";
          ofstream f(olp1.c_str(), ofstream::out);
          f << "# .PCD v0.7" << endl
            << "VERSION 0.7" << endl
            << "FIELDS x y z" << endl
            << "SIZE 4 4 4" << endl
            << "TYPE F F F" << endl
            << "COUNT 1 1 1" << endl
            << "WIDTH " << point_cloud_ptr->size() << endl
            << "HEIGHT 1" << endl
            << "VIEWPOINT 0 0 0 0 0 0 1" << endl
            << "POINTS " << point_cloud_ptr->size() << endl
            << "DATA ascii" << endl;
          for (size_t i = 0; i < point_cloud_ptr->size(); i++)
              f << point_cloud_ptr->points[i].x << " " << point_cloud_ptr->points[i].y  << " " << point_cloud_ptr->points[i].z  << endl;
          f.close();

  ///--------------------------------------------




  return (0);
}
