#include <cstdlib>
#include <iostream>
#include <fstream>
#include <list>
#include <cmath>
#include <sys/time.h>
#include "rp_loc/dmap_localizer.h"
#include "rp_base/draw_helpers.h"
#include "rp_base/grid_map.h"

using namespace std;
using Eigen::Isometry2f;
using Eigen::Rotation2Df;

list<double> compute_distance_map(string filename, float resolution, float dmax) {
  // load the map
  GridMap grid_map(0,0,resolution);
  grid_map.loadFromImage(filename.c_str(), resolution);

  DMapLocalizer localizer;
  localizer.setMap(grid_map, dmax);
  cerr << "localizer ready" << endl;
  cerr << "rows:  " << localizer.distances.rows << " cols: " << localizer.distances.cols << endl;

  // prepare canvas for visualization
  Canvas canvas;
  const auto& distances = localizer.distances;
  Grid_<uint8_t> image(distances.rows, distances.cols);
  
  // compute normalization of the DMAP
  float f_min=1e9;
  float f_max=0;
  for(auto& f: distances.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  float scale = 255./(f_max-f_min);



  list<double> obstacles_distances;
  for (size_t i=0; i<distances.cells.size(); ++i) {
    obstacles_distances.push_front(scale  * (distances.cells[i] - f_min));
  }

  return obstacles_distances;
}
