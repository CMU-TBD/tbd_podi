#ifndef TBD_COSTMAP_OPEN_SPACE_LAYER_H_
#define TBD_COSTMAP_OPEN_SPACE_LAYER_H_

#include <ros/ros.h>
#include <cmath>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <tbd_podi_2dnav/HumanLayerPluginConfig.h>
#include <tbd_podi_2dnav/OpenSpaceLayerPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

namespace tbd_costmap
{
class CellData
{
  public:
    CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy):
        index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy){}

    unsigned int index_;
    unsigned int x_, y_;
    unsigned int src_x_, src_y_;
};

class OpenSpaceLayer : public costmap_2d::Layer{
public:
  OpenSpaceLayer();

  virtual ~OpenSpaceLayer()
  {
    if (dsrv_)
        delete dsrv_;
    if (seen_)
        delete[] seen_;
  }

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual bool isDiscretized() {
    return true;
  }
  virtual void matchSize();

  virtual void reset() {
    onInitialize();
  }

// smootherstep function
  virtual inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;

    double euclidean_distance = distance * resolution_;

    if(euclidean_distance <= range_pref_){
        cost = 0;
    }
    else if(euclidean_distance >= range_max_){
        cost = (unsigned char)(cost_weight_*costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
    else {
    //  cost = (unsigned char)(cost_weight_*costmap_2d::INSCRIBED_INFLATED_OBSTACLE*0.5);
        double distance_scaled = (euclidean_distance - range_pref_)/(range_max_ - range_pref_);
        //cost = (unsigned char) (cost_weight_*costmap_2d::INSCRIBED_INFLATED_OBSTACLE*distance_scaled);
        cost = (unsigned char)(cost_weight_*costmap_2d::INSCRIBED_INFLATED_OBSTACLE*(6*pow(distance_scaled, 5) - 15*pow(distance_scaled, 4) + 10*pow(distance_scaled, 3)));
    }
    return cost;
  }

  void setDistanceParameters(double range_preferred, double range_max, double cost_weight);

protected:
  boost::recursive_mutex* open_space_access_;

  double resolution_;
  double range_pref_, range_max_, cost_weight_;

private:
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  }

  unsigned int cellDistance(double world_dist) {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);
  void computeCaches();

  std::map<double, std::vector<CellData> > open_cells_;

  bool* seen_;
  int seen_size_;
  bool recalculate_;

  unsigned char** cached_costs_;
  double** cached_distances_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  dynamic_reconfigure::Server<tbd_podi_2dnav::OpenSpaceLayerPluginConfig> *dsrv_;
  void reconfigureCB(tbd_podi_2dnav::OpenSpaceLayerPluginConfig &config, uint32_t level);

};

}  // namespace tbd_costmap

#endif