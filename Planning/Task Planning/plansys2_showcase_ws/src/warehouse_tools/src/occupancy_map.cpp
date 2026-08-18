#include "warehouse_tools/occupancy_map.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace warehouse_tools
{

namespace
{

// Reads the next PGM header token, skipping '#' comments to end of line. The
// maps this workspace consumes are written by `map_saver`, which puts its
// resolution in a comment on line two, so comment handling is not optional.
std::string nextToken(std::istream & in)
{
  std::string token;
  while (in >> token) {
    if (token[0] == '#') {
      std::string discard;
      std::getline(in, discard);
      continue;
    }
    return token;
  }
  throw std::runtime_error("truncated PGM header");
}

}  // namespace

OccupancyMap OccupancyMap::load(const std::string & yaml_path)
{
  YAML::Node meta;
  try {
    meta = YAML::LoadFile(yaml_path);
  } catch (const std::exception & error) {
    throw std::runtime_error("cannot read map metadata " + yaml_path + ": " + error.what());
  }

  const auto slash = yaml_path.find_last_of('/');
  const std::string dir = slash == std::string::npos ? "." : yaml_path.substr(0, slash);
  const std::string image = dir + "/" + meta["image"].as<std::string>();

  std::ifstream in(image, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot open map image " + image);
  }
  const std::string magic = nextToken(in);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error(image + ": expected a PGM (P2 or P5), found " + magic);
  }

  OccupancyMap map;
  map.width_ = std::stoi(nextToken(in));
  map.height_ = std::stoi(nextToken(in));
  const int max_value = std::stoi(nextToken(in));
  if (max_value <= 0 || max_value > 255) {
    throw std::runtime_error(image + ": unsupported maximum grey value");
  }

  std::vector<uint8_t> pixels(static_cast<size_t>(map.width_) * map.height_);
  if (magic == "P5") {
    in.get();   // the single whitespace byte that ends the header
    in.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (in.gcount() != static_cast<std::streamsize>(pixels.size())) {
      throw std::runtime_error(image + ": truncated raster");
    }
  } else {
    for (auto & pixel : pixels) {
      int value = 0;
      if (!(in >> value)) {
        throw std::runtime_error(image + ": truncated raster");
      }
      pixel = static_cast<uint8_t>(value);
    }
  }

  map.resolution_ = meta["resolution"].as<double>();
  map.origin_x_ = meta["origin"][0].as<double>();
  map.origin_y_ = meta["origin"][1].as<double>();
  const double free_thresh = meta["free_thresh"].as<double>();
  const bool negate = meta["negate"].as<int>() != 0;

  // map_server's convention: occupancy = (255 - value) / 255, inverted when
  // `negate` is set. Only the free class matters here — the unknown band is
  // treated as not-free, which is the conservative reading for a planner that
  // will drive through whatever it calls free.
  map.free_.assign(pixels.size(), 0);
  for (size_t i = 0; i < pixels.size(); ++i) {
    double occupancy = (255.0 - static_cast<double>(pixels[i])) / 255.0;
    if (negate) {
      occupancy = 1.0 - occupancy;
    }
    map.free_[i] = occupancy < free_thresh ? 1 : 0;
  }
  return map;
}

void OccupancyMap::toCell(const Point2D & p, int & col, int & row) const
{
  // The PGM's first row is the top of the map, so row indices count down from
  // the top while world y counts up from the origin.
  col = static_cast<int>(std::lround((p.x - origin_x_) / resolution_));
  row = height_ - 1 - static_cast<int>(std::lround((p.y - origin_y_) / resolution_));
}

void OccupancyMap::computeClearance(double radius)
{
  radius_ = radius;
  const size_t n = free_.size();
  const float infinity = std::numeric_limits<float>::infinity();
  distance_.assign(n, infinity);

  // Chamfer distance transform, 3-4 kernel scaled to metres. Two sweeps over
  // the grid; exact to within a few percent of Euclidean, which is finer than
  // the difference any waypoint decision here turns on.
  const float straight = static_cast<float>(resolution_);
  const float diagonal = static_cast<float>(resolution_ * std::sqrt(2.0));

  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      if (!free_[index(col, row)]) {
        distance_[index(col, row)] = 0.0f;
      }
    }
  }

  auto relax = [&](int col, int row, int dc, int dr, float cost) {
      const int nc = col + dc, nr = row + dr;
      if (!inside(nc, nr)) {
        return;
      }
      const float candidate = distance_[index(nc, nr)] + cost;
      if (candidate < distance_[index(col, row)]) {
        distance_[index(col, row)] = candidate;
      }
    };

  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      relax(col, row, -1, 0, straight);
      relax(col, row, 0, -1, straight);
      relax(col, row, -1, -1, diagonal);
      relax(col, row, 1, -1, diagonal);
    }
  }
  for (int row = height_ - 1; row >= 0; --row) {
    for (int col = width_ - 1; col >= 0; --col) {
      relax(col, row, 1, 0, straight);
      relax(col, row, 0, 1, straight);
      relax(col, row, 1, 1, diagonal);
      relax(col, row, -1, 1, diagonal);
    }
  }
}

double OccupancyMap::clearanceAt(const Point2D & p) const
{
  int col = 0, row = 0;
  toCell(p, col, row);
  if (!inside(col, row)) {
    return 0.0;
  }
  return static_cast<double>(distance_[index(col, row)]);
}

bool OccupancyMap::isClear(const Point2D & p) const
{
  int col = 0, row = 0;
  toCell(p, col, row);
  if (!inside(col, row)) {
    return false;
  }
  return free_[index(col, row)] && distance_[index(col, row)] >= radius_;
}

bool OccupancyMap::isSegmentClear(const Point2D & a, const Point2D & b) const
{
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  const int steps = std::max(2, static_cast<int>(std::ceil(length / (0.5 * resolution_))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    if (!isClear({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)})) {
      return false;
    }
  }
  return true;
}

}  // namespace warehouse_tools
