#ifndef GRID_MAPPING_GRID_HPP_
#define GRID_MAPPING_GRID_HPP_


#include "grid_mapping/grid.h"
#include <ros/ros.h>


namespace grid_mapping {


template <class T>
Grid<T>::Grid(Point origin_, double res, int w_, int h_, bool alloc_data) :
  GridBase(origin_, res, w_, h_)
{
  if (alloc_data)
    data = std::vector<T>(w*h, 0.0);
}


template <class T>
Grid<T>::Grid(const nav_msgs::OccupancyGrid::ConstPtr& msg) :
  GridBase(Point(msg->info.origin.position.x, msg->info.origin.position.y),
      msg->info.resolution, msg->info.width, msg->info.height)
{
  data.reserve(msg->data.size());
  for (auto cell : msg->data)
    data.push_back(static_cast<T>(cell));
}


// update existing map to have new dimensions
template <class T>
void Grid<T>::update(const Point new_origin, const int w_new, const int h_new)
{
  Grid<T> new_grid(new_origin, resolution, w_new, h_new);
  new_grid.update(this);
  origin = new_origin;
  w = w_new;
  h = h_new;
  data = new_grid.data;
}


// update existing map with other map info
template <class T>
void Grid<T>::update(const Grid* grid)
{
  int w_in = grid->w;
  int origin_offset = positionToIndex(grid->origin);
  for (int i = 0; i < grid->h; ++i) {
    int c = origin_offset + i*w;
    int c_in = i*w_in;
    for (int j = 0; j < w_in; ++j) {
      data[c+j] += grid->data[c_in + j];
    }
  }
}


// Expand map to include p_min, p_max
template <class T>
void Grid<T>::expandMap(const Point p_min, const Point p_max)
{
  // determine new extents of map, adding a padding of cells to attempt to
  // decrease the number of calls to expandMap(...), and subsequent copying
  Point new_origin(origin);
  Point new_top_corner = topCorner();
  double pad = round(0.2*std::max(w,h)) * resolution;
  if (p_min.x < new_origin.x)
    new_origin.x = roundToMapRes(p_min.x) - pad;
  if (p_min.y < new_origin.y)
    new_origin.y = roundToMapRes(p_min.y) - pad;
  if (p_max.x >= new_top_corner.x)
    new_top_corner.x = roundToMapRes(p_max.x) + pad;
  if (p_max.y >= new_top_corner.y)
    new_top_corner.y = roundToMapRes(p_max.y) + pad;
  int w_new = round((new_top_corner.x - new_origin.x) / resolution) + 1;
  int h_new = round((new_top_corner.y - new_origin.y) / resolution) + 1;

  // overwrite old map with new map
  update(new_origin, w_new, h_new);
}


template <class H>
std::ostream& operator<<(std::ostream& out, const Grid<H>& grid)
{
  std::cout << std::endl;
  std::cout << "info:" << std::endl;
  std::cout << "  origin: " << grid.origin << std::endl;
  std::cout << "  w: " << grid.w << std::endl;
  std::cout << "  h: " << grid.h << std::endl;
  std::cout << "  resolution: " << grid.resolution << std::endl;
  std::cout << "data:" << std::endl;
  for (int i = 0; i < grid.data.size(); ++i) {
    if (i % grid.w != 0)
      std::cout << grid.data[i] << ", ";
    else
      std::cout << std::endl << "  " << grid.data[i] << ", ";
  }
  std::cout << std::endl;
}


} // namespace grid_mapping


#endif
