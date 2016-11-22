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
#include <cmath>
#include <cstdlib>
#include <cassert>

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace Eigen;


using namespace std;
using namespace octomap;



void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}
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
void print4x4Matrix (double matrix[4][4])
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix[0][0], matrix[0][1], matrix[0][2]);
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix[1][0], matrix[1][1], matrix[1][2]);
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix[2][0], matrix[2][1], matrix[2][2]);
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix[0][3], matrix[1,3], matrix[2][3]);
}

void mnozenie3x3(double A[3][3], double B[3][3], double C[3][3]) {
  int i, j, k;

  for(i = 0; i < 3; i++)
    for(j = 0; j < 3; j++)
      for(k = 0; k < 3; k++)
        C[i][j] = C[i][j] + A[i][k] * B[k][j];

 // return 0;
}



int main (int argc, char** argv)
{
    cout<<"\nNo witam"<<endl;

    pcl::PointCloud<pcl::PointXYZ> cloud_a ;//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 *cloud2_a = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ> cloud_b ;//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 *cloud2_b = new pcl::PCLPointCloud2;

    pcl::PointCloud<pcl::PointXYZ> cloud_c ;//(new pcl::PointCloud<pcl::PointXYZ>);
   // pcl::PCLPointCloud2 *cloud2_c = new pcl::PCLPointCloud2;



    string infilename1 = "cloud00013.pcd";
    string infilename2 = "cloud00014.pcd";
    string outfilename = "nowy.pcd";

    if(pcl::io::loadPCDFile(infilename1, *cloud2_a) == -1) //load the file
    {
        cout<<"nie no tak tego nie wczytamy jedynki w sensie\n";
        return (-1);
    }
    if(pcl::io::loadPCDFile(infilename2, *cloud2_b) == -1) //load the file
    {
        cout<<"nie no tak tego nie wczytamy dwojki \n";
        return (-1);
    }

    cout<<"konwersja typu\n\n";

    pcl::fromPCLPointCloud2 (*cloud2_a,cloud_a);
    pcl::fromPCLPointCloud2 (*cloud2_b,cloud_b);

    
    
    
    ///NO spoko ale nie o to chodzi xd
     /*
    for(int i=0;i<cloud_b.size();i++)
    {
        cloud_b.points[i].x += 10;
        cloud_b.points[i].y += 0;
        cloud_b.points[i].z += 0;

    }
     */
    ///-----------------------------------------------

    int P;
    double point1[4];
    double point2[4];

    double kx = -M_PI/6;
    double ky = M_PI/6;
    double kz = M_PI/2;

    double tx = -0.6;
    double ty = -0.45;
    double tz = 0.4;

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


    print4x4Matrix(PA);

    Vector3d a1(cloud_b.points[50001].x,cloud_b.points[50001].y,cloud_b.points[50001].z);
    Vector3d a2(cloud_b.points[100001].x,cloud_b.points[100001].y,cloud_b.points[100001].z);
    Vector3d a3(cloud_b.points[150001].x,cloud_b.points[150001].y,cloud_b.points[150001].z);
    Vector3d a4(cloud_b.points[200001].x,cloud_b.points[200001].y,cloud_b.points[200001].z);
    Vector3d a5(cloud_b.points[75001].x,cloud_b.points[75001].y,cloud_b.points[75001].z);
    Vector3d a6(cloud_b.points[125001].x,cloud_b.points[125001].y,cloud_b.points[125001].z);
    Vector3d a7(cloud_b.points[175001].x,cloud_b.points[175001].y,cloud_b.points[175001].z);
    Vector3d a8(cloud_b.points[250001].x,cloud_b.points[250001].y,cloud_b.points[250001].z);

    for(int i=0;i<cloud_b.size();i++)
    {
      //point4d point ( cloud->points[i].x , cloud->points[i].y , cloud->points[i].z , 1 );

        double P2[4][1] = {
            {cloud_b.points[i].x},
            {cloud_b.points[i].y},
            {cloud_b.points[i].z},
            {1}};

        /* Mnozenie macierzy A * B = C */
        mnozeniemac(PA, P2, P1);

        if(i % 50000 == 1){
       cout<<cloud_b.points[i].x<<"   "<<cloud_b.points[i].y<<"   "<<cloud_b.points[i].z<<endl;}

        // no i przypisujeme

       // P2[0][0] = P1[0][0];
     //   P2[1][0] = P1[1][0];
     //   P2[2][0] = P1[2][0];
     //   P2[3][0] = P1[3][0];

       // cout<<P1[0][0]<<"   "<<P1[1][0]<<"   "<<P1[2][0]<<endl<<endl;

        cloud_b.points[i].x = P1[0][0];
        cloud_b.points[i].y = P1[1][0];
        cloud_b.points[i].z = P1[2][0];


        if(i % 50000 == 1)
        {cout<<cloud_b.points[i].x<<"   "<<cloud_b.points[i].y<<"   "<<cloud_b.points[i].z<<endl<<endl;}

       // P1 = PA * [cloud_b.points[i].x , cloud_b.points[i].y , cloud_b.points[i].z , 1 ];
    }
    
    
    
    cout<<"size : "<< cloud_b.size() <<endl;
    
    

    Vector3d b1(cloud_b.points[50001].x,cloud_b.points[50001].y,cloud_b.points[50001].z);
    Vector3d b2(cloud_b.points[100001].x,cloud_b.points[100001].y,cloud_b.points[100001].z);
    Vector3d b3(cloud_b.points[150001].x,cloud_b.points[150001].y,cloud_b.points[150001].z);
    Vector3d b4(cloud_b.points[200001].x,cloud_b.points[200001].y,cloud_b.points[200001].z);
    Vector3d b5(cloud_b.points[75001].x,cloud_b.points[75001].y,cloud_b.points[75001].z);
    Vector3d b6(cloud_b.points[125001].x,cloud_b.points[125001].y,cloud_b.points[125001].z);
    Vector3d b7(cloud_b.points[175001].x,cloud_b.points[175001].y,cloud_b.points[175001].z);
    Vector3d b8(cloud_b.points[250001].x,cloud_b.points[250001].y,cloud_b.points[250001].z);

    Matrix<double,3,8> start,end;
    start.col(0)=a1;
    start.col(1)=a2;
    start.col(2)=a3;
    start.col(3)=a4;
    start.col(4)=a5;
    start.col(5)=a6;
    start.col(6)=a7;
    start.col(7)=a8;


    end.col(0)=b1;
    end.col(1)=b2;
    end.col(2)=b3;
    end.col(3)=b4;
    end.col(4)=b5;
    end.col(5)=b6;
    end.col(6)=b7;
    end.col(7)=b8;

    cout << Eigen::umeyama(start,end,true) << endl;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    return 0;
  /*  if (strcmp(argv[1], "-p") == 0)
    {
      cloud_c  = cloud_a;
      cloud_c += cloud_b;
    //  std::cerr << "Cloud C: " << std::endl;
    //  for (size_t i = 0; i < cloud_c.points.size (); ++i)
     //   std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;
    }
    cout<<"2\n";

              string olp3 = "polaczenie.pcd";
              ofstream f2(olp3.c_str(), ofstream::out);
              f2 << "# .PCD v0.7" << endl
                << "VERSION 0.7" << endl
                << "FIELDS x y z" << endl
                << "SIZE 4 4 4" << endl
                << "TYPE F F F" << endl
                << "COUNT 1 1 1" << endl
                << "WIDTH " << cloud_c.size() << endl
                << "HEIGHT 1" << endl
                << "VIEWPOINT 0 0 0 0 0 0 1" << endl
                << "POINTS " << cloud_c.size() << endl
                << "DATA ascii" << endl;
              for (size_t i = 0; i < cloud_c.size(); i++)
                  f2 << cloud_b.points[i].x << " " << cloud_b.points[i].y  << " " << cloud_b.points[i].z << endl;
              f2.close();

      ///--------------------------------------------

*/
}
