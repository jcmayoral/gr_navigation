
#ifndef MARKERSUTILS_H
#define MARKERSUTILS_H

#include<visualization_msgs/MarkerArray.h>
#include<vector>

namespace gr_topological_visualizer{
  class MapGenerator{
    public:
      MapGenerator(){

      }
      virtual ~MapGenerator(){

      }
      void calculateCenters(std::vector<std::pair<float,float> >& vector, int plane_cells, int width_cells, float cell_size){
          std::cout << cell_size << std::endl;
          for(int i =0; i < plane_cells;i++){
            for(int j =0; j < plane_cells;j++){
              vector.push_back(std::make_pair(i*cell_size,j*cell_size));
            }   
          }
      }
  };
};
#endif 