#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/observation_buffer.h>
#include <tbd_ros_msgs/HumanBodyArray.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <geometry_msgs/Point.h>

namespace tbd_costmap
{

    class HumanLayer : public costmap_2d::CostmapLayer
    {
    private:
        ros::Subscriber humansSub_;
        std::string topicName_;
        double inflation_;
        double keepTimeSec_;
        bool ignoreTimeStamp_;
        ros::Time lastMsgTime_;
        std::vector<geometry_msgs::Point> latestPoints_;
        std::vector<geometry_msgs::Point> previousPoints_;
        std::vector<geometry_msgs::Point> constructPolygons(geometry_msgs::Point center, double *min_x, double *min_y,
                                                            double *max_x, double *max_y);

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