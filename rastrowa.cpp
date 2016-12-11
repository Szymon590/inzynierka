#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <iomanip>      // std::setw
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace octomap;

/// Homogeneous representation of SE(3) rigid body transformations
typedef Eigen::Matrix<double,4,4> Mat44;

//std::vector< Eigen::Quaternion<double> > trajectoryRot;


void print_query_info(point3d query, OcTreeNode* node) {
 if (node != NULL) {
   cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
 }
 else
   cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

void saveMap2file(std::string filename, OcTree& tree)
{
    ///grid to ta mapa rastrowa !
 std::ofstream ofs(filename.c_str());
 ofs<<"M=ones(" << int((5.01+5)/0.015)<<"," << int((5.01+5)/0.015) << ");\n" ;/// ciezko powiedziec xd

 std::cout << "Final number of nodes: " << tree.calcNumNodes() << "\n";/// ile nodow w drzewie

 std::vector< std::vector<double> > grid(668, std::vector<double>(668));/// tworzenie grida jako wektor 668 na 668

   for (int i=0;i<668;i++){
     for (int j=0;j<668;j++){
        grid[i][j]=0;         /// ustawianie niskich wartosci i dalej bedziemy to przeszukiwać :P
     }
   }


cout<<"zapinamy pasy"<<endl;
int i;

   for (auto it = tree.begin_tree(); it!=tree.end_tree();it++){

       if (it->getOccupancy() > 0.5){

            const octomap::point3d vx_center = it.getCoordinate();

                    if ((vx_center.x()>-5 && vx_center.x()<5) && (vx_center.y()>-2 && vx_center.y()<8)){

                          if (grid[int((vx_center.x()+5)/0.015)][int((vx_center.y()+2)/0.015)]  <  vx_center.z())
                          grid[int((vx_center.x()+5)/0.015)][int((vx_center.y()+2)/0.015)]  =  vx_center.z();

                    }
                    else
                    {
                        cout<<"hehe";
                    }
       }
       else
       {
           i++;
       }


   }/// kluczowe dla nas nastepuje tu przypisanie czy zajmuje czy nie

        cout<<"liczba : "<<i<<endl;
        i=0;



   /// te ofs ogolnie to jest funkja zapisywania
 ofs<<"\nM=[";
 for (int i=0;i<668;i++){
     for (int j=0;j<668;j++){

       ofs << grid[i][j] << ", ";
     }
     ofs << "\n";
 }
 ofs<<"];\n";

   ofs<<"x=["<<"\n";
  for (float i=-5.0;i<5.01;i+=0.015){
    ofs << i << " ";
  }
  ofs << "];\n";
  ofs<<"y=["<<"\n";
  for (float i=-5.0;i<5.01;i+=0.015){
    ofs << i << " ";
  }
  ofs << "];\n";

 ofs<<"[X,Y]=meshgrid(x,y);"<<"\n";
 ofs<<"surf(X,Y,M);"<<"\n";
 ofs<<"xlabel('x [m]')"<<"\n";
 ofs<<"ylabel('y [m]')"<<"\n";
 ofs<<"zlabel('z [m]')"<<"\n";
 ofs<<"colormap('autumn');"<<"\n";
 ofs.close();
}

int main(int argc, char** argv) {

     cout << endl;
     cout << "generating example map" << endl;

     string inputFilename = "nowy.bt";

     OcTree* tree = new OcTree(0.1);
     if (!tree->readBinary(inputFilename)){
       OCTOMAP_ERROR("Could not open file, exiting.\n");
       exit(1);
     }



   std::cout << "save map\n";
  saveMap2file("ICLoctreeNoise0p98.ot", *tree);
  saveMap2file("ICLoctreeNoise0p98.m", *tree);
  saveMap2file("ICLoctreeNoise0p98.hot", *tree);
  saveMap2file("ICLoctreeNoise0p98.dat", *tree);
  tree->writeBinary("simple_tree.bt");



/*
 cout << endl;
 cout << "performing some queries:" << endl;

 point3d query (0., 0., 0.);
 OcTreeNode* result = tree.search (query);
 print_query_info(query, result);

 query = point3d(-1.,-1.,-1.);
 result = tree.search (query);
 print_query_info(query, result);

 query = point3d(1.,1.,1.);
 result = tree.search (query);
 print_query_info(query, result);


 cout << endl;
 tree.writeBinary("simple_tree.bt");
 cout << "wrote example file simple_tree.bt" << endl << endl;
 cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
 cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  */

}
