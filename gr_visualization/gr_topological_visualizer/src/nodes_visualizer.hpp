
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
      void calculateCenters(std::vector<std::pair<float,float> >& vector, int h_cells, int w_cells, float h_size, float w_size){
          for(int i =0; i < h_cells;i++){
            for(int j =0; j < w_cells;j++){
              vector.push_back(std::make_pair(i*h_size,j*w_size));
            }   
          }
      }
  };
};
#endif 