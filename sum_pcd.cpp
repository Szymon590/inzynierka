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



int main()
{
    ///****************ZAŁADOWANIE POINTCLOUDÓW*****************************************************************************

    pcl::PCLPointCloud2 *cloud2_a = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 *cloud2_b = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 *cloud2_c = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a (new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b (new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c (new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::PointCloud<pcl::PointXYZ>);


    string infilename1 = "/home/szymon/Pulpit/trasy/trasa 2/jeden.pcd";
    string infilename2 = "/home/szymon/Pulpit/trasy/trasa 2/dwa.pcd";
    string infilename3 = "/home/szymon/Pulpit/trasy/trasa1_v2/cloud00008.pcd";

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

    if (io::loadPCDFile(infilename3, *cloud2_c) == -1)
    {
        cout<<"nie wczytąło"<<endl;
        return(-1);
    }
    cout << "Załadowano pliki .pcd" << endl;

    fromPCLPointCloud2 (*cloud2_a, *cloud_a);
    fromPCLPointCloud2 (*cloud2_b, *cloud_b);
    fromPCLPointCloud2 (*cloud2_c, *cloud_c);

    cout << "Przekształcono Pointcloud2 do Pointcloud" << endl;



    ///***********************MACIERZE WEJŚCIOWE****************************************************************************
    double P1[4][1] = {
        {1},
        {1},
        {1},
        {1}};


    double PA1[4][4]{
        {    0.978807 , -0.0192573 , -0.0500585 , -0.0521736},
         {  0.0196949 ,   0.980045 , 0.00807992  ,  -0.48616},
          {  0.049888 ,-0.00907355  ,  0.978964  ,  0.322712},
           {        0   ,        0     ,      0     ,      1}};



\

    ///**********************W PĘTLI CLOUD_B JEST TRANSFORMOWANA O MACIERZ TRANSFORMACJI PA*********************************
    for(int i=0;i<cloud_b->size();i++)
    {
        double P2[4][1] = {
            {cloud_b->points[i].x},
            {cloud_b->points[i].y},
            {cloud_b->points[i].z},
            {1}};

        /* Mnozenie macierzy A * B = C*/
        mnozeniemac(PA1, P2, P1);

        cloud_b->points[i].x = P1[0][0];
        cloud_b->points[i].y = P1[1][0];
        cloud_b->points[i].z = P1[2][0];

    }

    double PA2[4][4]{
        {    0.977059 , -0.0357135   ,0.0276728  ,  0.613993},
         {   0.0359772 ,   0.977401, -0.00886793  , -0.023444},
          { -0.0273292  ,0.00987635  ,  0.977672 ,-0.00316171},
           {         0    ,       0   ,        0    ,       1}};


\

    ///**********************W PĘTLI CLOUD_B JEST TRANSFORMOWANA O MACIERZ TRANSFORMACJI PA*********************************
    for(int i=0;i<cloud_b->size();i++)
    {
        double P2[4][1] = {
            {cloud_c->points[i].x},
            {cloud_c->points[i].y},
            {cloud_c->points[i].z},
            {1}};

        /* Mnozenie macierzy A * B = C*/
        mnozeniemac(PA2, P2, P1);

        cloud_c->points[i].x = P1[0][0];
        cloud_c->points[i].y = P1[1][0];
        cloud_c->points[i].z = P1[2][0];

    }


    cout << "Pomyślnie wykonano transformację chmury punktów" << endl;




    //cloud_c += *cloud_a + *cloud_b;


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

    ///******************************WYŚWIETLANIE***************************************************************************
    cout << "Rozpoczynanie procedury wyświetlania wyników transformacji" << endl;

    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");

    pcl::visualization::PCLVisualizer viewer1 ("Matrix transformation example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1 (cloud_a, 255, 0, 0);
    viewer1.addPointCloud (cloud_a, source_cloud_color_handler1, "original_cloud1");

    viewer1.addCoordinateSystem (1.0, "cloud", 0);
    viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer1.spinOnce ();
    }

    cout << "Zakończono proces wizualizacji" << endl;

}
