#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/observation_buffer.h>
#include <tbd_ros_msgs/HumanBodyArray.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <map>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace tbd_costmap
{

    class HumanLayer : public costmap_2d::CostmapLayer
    {
    private:
        ros::Subscriber humansSub_;
        std::string topicName_;
        std::string operating_frame_id_;
        double inflation_;
        double keepTimeSec_;
        bool ignoreTimeStamp_;
        ros::Time lastMsgTime_;
        std::map<int, geometry_msgs::Point> latestPoints_;
        std::map<int, geometry_msgs::Point> previousPoints_;
        std::map<int, geometry_msgs::Quaternion> latestOrients_;
        std::map<int, geometry_msgs::Quaternion> previousOrients_;
        std::map<int, double> previousVelocities_;
        std::map<int, double> latestVelocities_;
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

        std::vector<geometry_msgs::Point> constructPolygons(geometry_msgs::Point center, geometry_msgs::Quaternion orient, double vel, double *min_x, double *min_y,
                                                            double *max_x, double *max_y);
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    public:
        HumanLayer();
        virtual ~HumanLayer();
        // Implement all the parent methods
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y);
        virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual void activate();
        virtual void deactivate();
        virtual void reset();

        void HumansCB(const tbd_ros_msgs::HumanBodyArray &msg);
    };

} // namespace tbd_costmap