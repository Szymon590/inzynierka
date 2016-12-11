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


/* Mnozenie macierzy A * B = C */        // xdd
void mnozeniemac(double A[4][4], double B[4][1], double C[4][1]) {
  int i, j, k;

  for(i = 0; i < 4; i++)
    for(j = 0; j < 1; j++)
      C[i][j] = 0;

  for(i = 0; i < 4; i++)
    for(j = 0; j < 1; j++)
      for(k = 0; k < 4; k++)
        C[i][j] = C[i][j] + A[i][k] * B[k][j];

 // return 0;
}

void mnozenie3x3(double A[3][3], double B[3][3], double C[3][3]) {
  int i, j, k;

  for(i = 0; i < 3; i++)
    for(j = 0; j < 3; j++)
      for(k = 0; k < 3; k++)
        C[i][j] = C[i][j] + A[i][k] * B[k][j];

 // return 0;
}



// This function displays the help
void showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

// This is the main function
int main (int argc, char** argv)
{

  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }
/*
  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }
*/
  // Load file | Works with PCD and PLY files
  //pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_23 (new pcl::PointCloud<pcl::PointXYZ>); // PTR
  pcl::PCLPointCloud2 *cloud2_23 = new pcl::PCLPointCloud2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_24 (new pcl::PointCloud<pcl::PointXYZ>); // PTR
  pcl::PCLPointCloud2 *cloud2_24 = new pcl::PCLPointCloud2;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr (new pcl::PointCloud<pcl::PointXYZ>); // PTR



  string infilename23 = "/home/szymon/Pulpit/trasy/trasa1_v2/dwa.pcd";
  string infilename24 = "/home/szymon/Pulpit/trasy/trasa1_v2/dwa.pcd";


     if(pcl::io::loadPCDFile(infilename23, *cloud2_23) == -1) //load the file
    {
      std::cout << "Error loading point cloud " << std::endl << std::endl;
      return -1;
    }
     if(pcl::io::loadPCDFile(infilename24, *cloud2_24) == -1) //load the file
    {
      std::cout << "Error loading point cloud " << std::endl << std::endl;
      return -1;
    }

     cout<<"konwersja typu\n\n";

    pcl::fromPCLPointCloud2 (*cloud2_23,*cloud_23);
    pcl::fromPCLPointCloud2 (*cloud2_24,*cloud_24);


 // } else {
 //   if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
 //     std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
 //     showHelp (argv[0]);
//      return -1;
 //   }
//  }

  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float ksi = M_PI/4; // Z
  float fi = 0; //X
  float theta = 0; //Y
/*
  transform_1 (0,0) = cos(fi)*cos(ksi)-(sin(fi)*sin(ksi)*cos(theta));
  transform_1 (0,1) = (-cos(fi)*sin(ksi)) - (sin(fi)*cos(ksi)*cos(theta));
  transform_1 (0,2) = sin(fi)*sin(theta);

  transform_1 (1,0) = sin(fi)*sin(ksi) + cos(fi)*sin(ksi)*cos(theta);
  transform_1 (1,1) = (-sin(fi)*sin(ksi)+cos(fi)*cos(ksi)*cos(theta));
  transform_1 (1,2) = (-cos(fi)*sin(theta));

  transform_1 (2,0) = sin(ksi)*sin(theta);
  transform_1 (2,1) = cos(ksi)*sin(theta);
  transform_1 (2,2) = cos(theta);
*/
  //    (row, column)
  // Define a translation of 2.5 meters on the x axis.
 // transform_1 (0,3) = 0;
  // Print the transformation
 //printf ("Method #1: using a Matrix4f\n");
  //std::cout << transform_1 << std::endl;

/*
  double kx = -M_PI/4;
  double ky = M_PI/3.2;
  double kz = M_PI/2;

  double tx = -2;
  double ty = -1.15;
  double tz = 1.15;
*/




  int P;
  double point1[4];
  double point2[4];

  double kx = -0.5 * M_PI;
  double ky = 0 * M_PI;
  double kz = 0;

  double tx = 0;
  double ty = 0;
  double tz = 0;

  double X[3][3] ={
      {1,0,0},
      {0,cos(kx),-sin(kx)},
      {0,sin(kx),cos(kx)}};
  double Y[3][3] ={
      {cos(ky),0,sin(ky)},
      {0,1,0},
      {-sin(ky),0,cos(ky)}};
  double Z[3][3] ={
      {cos(kz),-sin(kz),0},
      {sin(kz),cos(kz),0},
      {0,0,1}};


  double XY[3][3] ={
      {0,0,0},
      {0,0,0},
      {0,0,0}};
  double XYZ[3][3] ={
      {0,0,0},
      {0,0,0},
      {0,0,0}};
  mnozenie3x3(X,Y,XY);
  mnozenie3x3(XY,Z,XYZ);

  double PA[4][4] = {
    {  XYZ[0][0],   XYZ[0][1],   XYZ[0][2],   tx},
    {  XYZ[1][0],   XYZ[1][1],   XYZ[1][2],   ty},
    {  XYZ[2][0],   XYZ[2][1],   XYZ[2][2],   tz},
    {  0,   0,   0,   1}};
  double PA1[4][4] = {
    { 1,   0,   0,   0},
    {  0,   1,   0,   0},
    {  0,   0,   1,   0},
    {  0,   0,   0,   1}};

  double P1[4][1] = {
      {1},
      {1},
      {1},
      {1}};






  for(int i=0;i<cloud_24->size();i++)
  {
    //point4d point ( cloud->points[i].x , cloud->points[i].y , cloud->points[i].z , 1 );

      double P2[4][1] = {
          {cloud_24->points[i].x},
          {cloud_24->points[i].y},
          {cloud_24->points[i].z},
          {1}};

      /* Mnozenie macierzy A * B = C*/
      mnozeniemac(PA, P2, P1);

    // cout<<cloud_b.points[i].x<<"   "<<cloud_b.points[i].y<<"   "<<cloud_b.points[i].z<<endl;

      // no i przypisujeme

     // P2[0][0] = P1[0][0];
   //   P2[1][0] = P1[1][0];
   //   P2[2][0] = P1[2][0];
   //   P2[3][0] = P1[3][0];

     // cout<<P1[0][0]<<"   "<<P1[1][0]<<"   "<<P1[2][0]<<endl<<endl;

      cloud_24->points[i].x = P1[0][0];
      cloud_24->points[i].y = P1[1][0];
      cloud_24->points[i].z = P1[2][0];

    //  cout<<cloud_b.points[i].x<<"   "<<cloud_b.points[i].y<<"   "<<cloud_b.points[i].z<<endl<<endl;

     // P1 = PA * [cloud_b.points[i].x , cloud_b.points[i].y , cloud_b.points[i].z , 1 ];
  }





  // Executing the transformation
 // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
//  pcl::transformPointCloud (*cloud_24, *transformed_cloud, transform_1);


  //Dodawanie


  //cloud_23 += cloud_24;


  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer1 ("Matrix transformation example");
 // pcl::visualization::PCLVisualizer viewer2 ("Matrix transformation example");


   // Define R,G,B colors for the point cloud
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255.0, 255.0, 255.0);
  // We add the point cloud to the viewer and pass the color handler
  //viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");


 // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1 (cloud_23, 0, 0, 255);
 // viewer1.addPointCloud (cloud_23, source_cloud_color_handler1, "original_cloud1");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (cloud_24, 255, 0, 0);
  viewer1.addPointCloud (cloud_24, source_cloud_color_handler2, "original_cloud2");

  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  //viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

 // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (cloud_24, 255, 0, 0);
//  viewer2.addPointCloud (cloud_24, source_cloud_color_handler3, "original_cloud2");

  viewer1.addCoordinateSystem (1.0, "cloud", 0);
  viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//  viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  int v1(0);
  int v2(1);


 // viewer2.addCoordinateSystem (1.0, "cloud", 0);
 // viewer2.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
 // viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//  viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

  //viewer.setPosition(800, 400); // Setting visualiser window position


          string olp = "osZ.pcd";
          ofstream f(olp.c_str(), ofstream::out);
          f << "# .PCD v0.7" << endl
            << "VERSION 0.7" << endl
            << "FIELDS x y z" << endl
            << "SIZE 4 4 4" << endl
            << "TYPE F F F" << endl
            << "COUNT 1 1 1" << endl
            << "WIDTH " << cloud_24->size() << endl
            << "HEIGHT 1" << endl
            << "VIEWPOINT 0 0 0 0 0 0 1" << endl
            << "POINTS " << cloud_24->size() << endl
            << "DATA ascii" << endl;
          for (size_t i = 0; i < cloud_24->size(); i++)
              f << cloud_24->points[i].x << " " << cloud_24->points[i].y  << " " << cloud_24->points[i].z  << endl;
          f.close();


  while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer1.spinOnce ();


}
}
