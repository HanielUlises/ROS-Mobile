// A `map_server` occupancy grid, and the two questions a task planner has to
// ask of one.
//
// The symbolic layer is allowed to be abstract; the correspondence between it
// and the floor is not. Every waypoint this workspace emits has to satisfy
// "the vehicle fits here", and every edge has to satisfy "the vehicle can drive
// straight from here to there", both at the vehicle's own clearance. Those are
// the only two queries, and this class answers exactly them so that no other
// part of the workspace has to reason about pixels.

#ifndef WAREHOUSE_TOOLS__OCCUPANCY_MAP_HPP_
#define WAREHOUSE_TOOLS__OCCUPANCY_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace warehouse_tools
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

class OccupancyMap
{
public:
  // Loads the `map_server` pair (<stem>.yaml naming a PGM beside it). Throws
  // std::runtime_error with the offending file named, because a mis-specified
  // map is the kind of failure that otherwise shows up much later as an
  // unexplained "no plan".
  static OccupancyMap load(const std::string & yaml_path);

  // Free space eroded by `radius` metres. Cheap: a two-pass chamfer distance
  // transform over the occupied set, thresholded once. Called once per tool
  // run, so clarity beats cleverness.
  void computeClearance(double radius);

  bool isClear(const Point2D & p) const;

  // Samples the segment at half-cell steps: at 0.05 m/cell that is finer than
  // any obstacle the grid can represent, so a segment that passes cannot be
  // hiding a wall between samples.
  bool isSegmentClear(const Point2D & a, const Point2D & b) const;

  // Distance in metres from p to the nearest occupied cell (infinite if the map
  // holds no obstacle at all). Reported by the roadmap builder so that a
  // waypoint that only just fits is visible as such.
  double clearanceAt(const Point2D & p) const;

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}
  double originX() const {return origin_x_;}
  double originY() const {return origin_y_;}
  double radius() const {return radius_;}

private:
  bool inside(int col, int row) const
  {
    return col >= 0 && row >= 0 && col < width_ && row < height_;
  }
  size_t index(int col, int row) const {return static_cast<size_t>(row) * width_ + col;}
  void toCell(const Point2D & p, int & col, int & row) const;

  int width_{0};
  int height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double radius_{0.0};

  std::vector<uint8_t> free_;        // 1 where the map calls the cell free
  std::vector<float> distance_;      // metres to the nearest occupied cell
};

}  // namespace warehouse_tools

#endif  // WAREHOUSE_TOOLS__OCCUPANCY_MAP_HPP_
