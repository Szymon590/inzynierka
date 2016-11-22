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
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace octomap;
using namespace Eigen;

int main (int argc, char** argv)
{

    Vector3d a1(-1.0829,-0.8143,1.785);
    Vector3d a2(-0.681749,-0.443897,1.452);
    Vector3d a3(-0.264857,-0.18952,1.236);
    Vector3d a4(-0.0612333,0.007952,0.835);

    Vector3d b1(0.997704,-0.818466,1.9276);
    Vector3d b2(0.510426,-0.522651,1.63766);
    Vector3d b3(0.182129,-0.191549,1.37736);
    Vector3d b4(-0.183189,-0.141265,1.05721);

    Matrix<double,3,4> start,end;
    start.col(0)=a1;
    start.col(1)=a2;
    start.col(2)=a3;
    start.col(3)=a4;



    end.col(0)=b1;
    end.col(1)=b2;
    end.col(2)=b3;
    end.col(3)=b4;

    cout << Eigen::umeyama(start,end,true) << endl;

  return (0);
}
