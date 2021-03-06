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

    string infilename1 = "/home/szymon/Pulpit/trasy/trasa 2/jeden.pcd";
    string infilename2 = "/home/szymon/Pulpit/trasy/trasa 2/dwa.pcd";

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


    ///***********************MACIERZE WEJŚCIOWE****************************************************************************
    double P1[4][1] = {
        {1},
        {1},
        {1},
        {1}};


    double PA[4][4]{
    {    0.978807 , -0.0192573 , -0.0500585 , -0.0521736},
     {  0.0196949 ,   0.980045 , 0.00807992  ,  -0.48616},
      {  0.049888 ,-0.00907355  ,  0.978964  ,  0.322712},
       {        0   ,        0     ,      0     ,      1}};







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

    *cloud_a += *cloud_b;
  //  *cloud_a += *cloud_c;

    string olp = "trzy.pcd";
    ofstream f(olp.c_str(), ofstream::out);
    f << "# .PCD v0.7" << endl
      << "VERSION 0.7" << endl
      << "FIELDS x y z" << endl
      << "SIZE 4 4 4" << endl
      << "TYPE F F F" << endl
      << "COUNT 1 1 1" << endl
      << "WIDTH " << cloud_a->size() << endl
      << "HEIGHT 1" << endl
      << "VIEWPOINT 0 0 0 0 0 0 1" << endl
      << "POINTS " << cloud_a->size() << endl
      << "DATA ascii" << endl;
    for (size_t i = 0; i < cloud_a->size(); i++)
        f << cloud_a->points[i].x << " " << cloud_a->points[i].y  << " " << cloud_a->points[i].z  << endl;
    f.close();


    while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer1.spinOnce ();
    }
    cout << "Zakończono proces wizualizacji" << endl;

    return 0;
}
