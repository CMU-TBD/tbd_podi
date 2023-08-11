#include <algorithm>
#include <tbd_podi_2dnav/open_space_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(tbd_costmap::OpenSpaceLayer, costmap_2d::Layer)

using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace tbd_costmap
{

OpenSpaceLayer::OpenSpaceLayer():
    resolution_(0),
    dsrv_(NULL), seen_(NULL), cached_costs_(NULL), cached_distances_(NULL),
    last_min_x_(-std::numeric_limits<float>::max()), last_min_y_(-std::numeric_limits<float>::max()), 
    last_max_x_(std::numeric_limits<float>::max()), last_max_y_(std::numeric_limits<float>::max())
{
  open_space_access_ = new boost::recursive_mutex();
}

void OpenSpaceLayer::onInitialize(){
    boost::unique_lock < boost::recursive_mutex > lock(*open_space_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    recalculate_ = false;

    nh.param("range_preferred", range_pref_, 1.0);
    nh.param("range_max", range_max_, 2.0);
    nh.param("cost_weight", cost_weight_, 0.5);

    dynamic_reconfigure::Server<tbd_podi_2dnav::OpenSpaceLayerPluginConfig>::CallbackType cb =
        [this](auto& config, auto level){
            reconfigureCB(config, level);
    };

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<tbd_podi_2dnav::OpenSpaceLayerPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }

  matchSize();
}

void OpenSpaceLayer::reconfigureCB(tbd_podi_2dnav::OpenSpaceLayerPluginConfig &config, uint32_t level)
{
  setDistanceParameters(config.range_preferred, config.range_max, config.cost_weight);

  if (enabled_ != config.enabled) {
    enabled_ = config.enabled;
    recalculate_ = true;
  }
}

void OpenSpaceLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*open_space_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];

  computeCaches();
}

void OpenSpaceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
}

void OpenSpaceLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*open_space_access_);

  ROS_ASSERT_MSG(open_cells_.empty(), "The list must be empty at the beginning of adding costs");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_ == NULL) {
    ROS_WARN("OpenSpaceLayer::updateCosts(): seen_ array is NULL");
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    ROS_WARN("OpenSpaceLayer::updateCosts(): seen_ array size is wrong");
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  std::vector<CellData>& obs_bin = open_cells_[0.0];
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      {
        obs_bin.push_back(CellData(index, i, j, i, j));
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells

  std::map<double, std::vector<CellData> >::iterator bin;
  for (bin = open_cells_.begin(); bin != open_cells_.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i)
    {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      master_array[index] = std::max(old_cost, cost);

      // attempt to put the neighbors of the current cell onto the list
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);
    }
  }

  open_cells_.clear();
}

inline void OpenSpaceLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index])
  {
    double distance = distanceLookup(mx, my, src_x, src_y);

    // push the cell data onto the list and mark
    open_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}

void OpenSpaceLayer::computeCaches()
{
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  unsigned int rad_max = std::max(size_x, size_y);

  cached_costs_ = new unsigned char*[rad_max + 1];
  cached_distances_ = new double*[rad_max + 1];

    for (unsigned int i = 0; i <= rad_max + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[rad_max + 2];
      cached_distances_[i] = new double[rad_max + 2];
      for (unsigned int j = 0; j <= rad_max + 1; ++j)
      {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

  for (unsigned int i = 0; i <= rad_max + 1; ++i)
  {
    for (unsigned int j = 0; j <= rad_max + 1; ++j)
    {
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void OpenSpaceLayer::setDistanceParameters(double range_preferred, double range_max, double cost_weight)
{
  if (range_pref_ != range_preferred || range_max_ != range_max || cost_weight_ != cost_weight)
  {
    // Lock here so that reconfiguring the radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*open_space_access_);

    range_pref_ = range_preferred;
    range_max_ = range_max;
    cost_weight_ = cost_weight;
    recalculate_ = true;
    computeCaches();
  }

}

}  // namespace tbd_costmap