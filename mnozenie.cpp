#include <iostream> // std::cout
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


using namespace std;
using namespace octomap;
using namespace pcl;

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

int main (int argc, char** argv)
{
    ///****************ZAŁADOWANIE POINTCLOUDÓW*****************************************************************************

    pcl::PCLPointCloud2 *cloud2_a = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 *cloud2_b = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a (new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b (new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::PointCloud<pcl::PointXYZ>);

    string infilename1 = "/home/szymon/Pulpit/Programik/dzialania wojenne/cloud00043.pcd";
    string infilename2 = "/home/szymon/Pulpit/Programik/dzialania wojenne/cloud00044.pcd";

    if (io::loadPCDFile(infilename1, *cloud2_a)==-1)
    {
        cout << "cannot load the file" << endl;
        return (-1);
    }

    if (io::loadPCDFile(infilename2, *cloud2_b)==-1)
    {
        cout << "cannot load the file" << endl;
        return (-1);
    }
    cout << "Załadowano pliki .pcd" << endl;

    fromPCLPointCloud2 (*cloud2_a, *cloud_a);
    fromPCLPointCloud2 (*cloud2_b, *cloud_b);

    cout << "Przekształcono Pointcloud2 do Pointcloud" << endl;

    ///***********************DANE DO MACIERZY TRANSFORMACJI****************************************************************
    double kx = -M_PI/4;
    double ky = M_PI/3.2;
    double kz = M_PI/2;

    double tx = -2;
    double ty = -1.15;
    double tz = 1.15;

    ///***********************MACIERZE WEJŚCIOWE****************************************************************************
    double P1[4][1] = {
        {1},
        {1},
        {1},
        {1}};


    double PA[4][4]{
      {  1, 0, 2, 2}, //612.588},
      {  1, 5, 8, 0},//310.611},
      { -2, 0, 0, 1}, //0.59661},
      {  0, 0, 0, 1}};

    ///**********************W PĘTLI CLOUD_B JEST TRANSFORMOWANA O MACIERZ TRANSFORMACJI PA*********************************
    for(int i=0;i<cloud_b->size();i++)
    {
        double P2[4][1] = {
            {cloud_b->points[i].x},
            {cloud_b->points[i].y},
            {cloud_b->points[i].z},
            {1}};

        if(i % 50000 == 1)
        {cout<<cloud_b->points[i].x<<"   "<<cloud_b->points[i].y<<"   "<<cloud_b->points[i].z<<endl;}

        /* Mnozenie macierzy A * B = C*/
        mnozeniemac(PA, P2, P1);

        cloud_b->points[i].x = P1[0][0];
        cloud_b->points[i].y = P1[1][0];
        cloud_b->points[i].z = P1[2][0];

        if(i % 50000 == 1)
        {cout<<cloud_b->points[i].x<<"   "<<cloud_b->points[i].y<<"   "<<cloud_b->points[i].z<<endl<<endl;}

    }
    cout << "Pomyślnie wykonano transformację chmury punktów" << endl;

    ///******************************WYŚWIETLANIE***************************************************************************
    cout << "Rozpoczynanie procedury wyświetlania wyników transformacji" << endl;

    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");

    pcl::visualization::PCLVisualizer viewer1 ("Matrix transformation example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1 (cloud_a, 0, 0, 255);
    viewer1.addPointCloud (cloud_a, source_cloud_color_handler1, "original_cloud1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (cloud_b, 255, 0, 0);
    viewer1.addPointCloud (cloud_b, source_cloud_color_handler2, "original_cloud2");

    viewer1.addCoordinateSystem (1.0, "cloud", 0);
    viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    int v1(0);
    int v2(1);


    while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer1.spinOnce ();
    }
    cout << "Zakończono proces wizualizacji" << endl;

    return 0;
}
