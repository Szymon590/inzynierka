/*
                <Szymi Programs>
 W tym pliku zaimportuje najpierw pojedyńcze zdjęcie z formaem pcd,
 a potem popracuje nad jednorazowym importem całego folderu.

 */

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

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
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



int main(int argc, char** argv){

    cout<<"\nNo witam"<<endl;

    /*tworze PointClouda i PointCloud2, z niewiadomych przyczyn przy importowaniu chmury do PointCloud
    mamy błąd z pamięcia, natomiast PointCloud2 nie ma wielu fukncji dlatego ostatecznie konwertujemy go do PointCloud
    */

    pcl::PointCloud<pcl::PointXYZ> *cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;

    string infilename = "osZ.pcd";
    string outfilename = "nowy.bt";

    if(pcl::io::loadPCDFile(infilename, *cloud2) == -1) //load the file
    {
        cout<<"nie no tak tego nie wczytamy\n";
        return (-1);
    }

    cout<<"konwersja typu\n\n";

    pcl::fromPCLPointCloud2 (*cloud2,*cloud);

    cout<<"tworzenie octree\n";

    OcTree tree(0.005); //nie do konca czaje tą rozdzielczosc, im mniejsza tym wiecej nodów ale co to xd

    for(int i=0;i<cloud->size();i++)
    {
       point3d endpoint (cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
       tree.updateNode(endpoint,true);
    }



    cout<<"ilosc Nodów zajętych"<<tree.size()<<"\n\n";

    point3d query (0., 0., 0.);
    OcTreeNode* result = tree.search (query);
    print_query_info(query, result);

    query = point3d(-1.,-1.,-1.);
    result = tree.search (query);               // co to robi xd
    print_query_info(query, result);

    query = point3d(1.,1.,1.);
    result = tree.search (query);
    print_query_info(query, result);


    cout << endl;
    tree.writeBinary(outfilename);
    cout << "wrote example file "<< outfilename << endl << endl;
    cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
    cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;



    ///ZAPISYWANIE DO PCD
    /*
            string olp = "nowy.pcd";
            ofstream f(olp.c_str(), ofstream::out);
            f << "# .PCD v0.7" << endl
              << "VERSION 0.7" << endl
              << "FIELDS x y z" << endl
              << "SIZE 4 4 4" << endl
              << "TYPE F F F" << endl
              << "COUNT 1 1 1" << endl
              << "WIDTH " << cloud->size() << endl
              << "HEIGHT 1" << endl
              << "VIEWPOINT 0 0 0 0 0 0 1" << endl
              << "POINTS " << cloud->size() << endl
              << "DATA ascii" << endl;
            for (size_t i = 0; i < cloud->size(); i++)
                f << cloud->points[i].x << " " << cloud->points[i].y  << " " << cloud->points[i].z  << endl;
            f.close();
    */
    ///--------------------------------------------

    return 1;

}



