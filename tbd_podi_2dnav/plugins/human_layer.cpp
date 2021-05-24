
#include <tbd_podi_2dnav/human_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(tbd_costmap::HumanLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace tbd_costmap
{
    HumanLayer::HumanLayer()
        : dsrv_(NULL)
    {
        costmap_ = NULL;

    }

    HumanLayer::~HumanLayer()
    {
        if (dsrv_)
        {
            delete dsrv_;
        }
    }

    void HumanLayer::onInitialize()
    {
        ros::NodeHandle n;
        ros::NodeHandle nh("~/" + name_);

        //initialize it
        HumanLayer::setDefaultValue(costmap_2d::FREE_SPACE);
        HumanLayer::matchSize();
        //enable the map

        current_ = true;

        // get the properities
        nh.param("topic", topicName_, std::string("/humans"));
        nh.param("inflation", inflation_, 0.5);
        nh.param("ignore_time_stamp", ignoreTimeStamp_, false);
        nh.param("observation_keep_time", keepTimeSec_, 1.0);
        nh.param("enabled", enabled_, true);

        // get information about the cost map
        operating_frame_id_ = layered_costmap_->getGlobalFrameID();

        // subscribe to the humans topic
        humansSub_ = n.subscribe(topicName_, 1, &HumanLayer::HumansCB, this);

        // start dynamic reconfigure
        // modelled after from https://github.com/ros-planning/navigation
        if (dsrv_)
        {
            delete dsrv_;
        }
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &HumanLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void HumanLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        if (config.enabled != enabled_)
        {
            enabled_ = config.enabled;
        }
    }

    std::vector<geometry_msgs::Point> HumanLayer::constructPolygons(geometry_msgs::Point point, geometry_msgs::Quaternion orient, double vel, double *min_x, double *min_y,
                                                                    double *max_x, double *max_y)
    {
        // the polygon around the human to be returned
        std::vector<geometry_msgs::Point> polygon;
        
        // stores the value of pi
        double pi = atan(1)*4;

        // get RPY from quaternion
        // convert geometry_msgs quat to tf2 quat to get angles
        tf2::Quaternion quat_tf;
        tf2::convert(orient, quat_tf);
        
        // convert the input quarernions into RPY
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

        // set the reference angle
        yaw -= pi;        

        // loop and get 8 points for a ellipse type polygon
        for (int i = 0; i < 8; i++)
        {
            geometry_msgs::Point curr_point(point);

            double curr_angle = 2 * pi * (i /(double)8);

            // major and minor diameters for the ellipse
            double a = (inflation_/(double)2);
            double b = inflation_ + vel; // changes based on velocity


            // ellipse offset to that their is more space in front of the person
            double offset = (b/(double)3) + vel*3;

            // gets x and y for an ellipse, the last term is for the rotation at an offset point (found graphically using desmos)
            // these are the parametric equations for the ellipse around the person
            // you can plug these into desmos to see them in action
            curr_point.x += a * cos(curr_angle) * cos(yaw) - b * sin(curr_angle) * sin(yaw) + offset * cos(yaw + (pi/(double)2));
            curr_point.y += a * cos(curr_angle) * sin(yaw) + b * sin(curr_angle) * cos(yaw) + offset * sin(yaw + (pi/(double)2));

            // add to the polygon
            polygon.push_back(curr_point);
        }

        *min_x = std::min(point.x - inflation_, *min_x);
        *min_y = std::min(point.y - inflation_, *min_y);
        *max_x = std::max(point.x + inflation_, *max_x);
        *max_y = std::max(point.y + inflation_, *max_y);
        return polygon;
    }

    void HumanLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y)
    {
        // clear the previous polygons
        if (previousPoints_.size() > 0 && previousOrients_.size() > 0)
        {
            for (auto &item : previousPoints_)
            {

                auto body_id = item.first;
                auto orient = previousOrients_[body_id];
                auto point = item.second;

                // get the velocity if calculated previously
                double vel = 0.0;
                if (previousVelocities_.find(body_id) != previousVelocities_.end())
                {
                    vel = previousVelocities_[body_id];
                }

                auto polygon = constructPolygons(point, orient, vel, min_x, min_y, max_x, max_y);
                setConvexPolygonCost(polygon, costmap_2d::FREE_SPACE);

            }
        }

        if ( ignoreTimeStamp_ || (lastMsgTime_ + ros::Duration(keepTimeSec_)) > ros::Time::now())
        {         

            // loop thorugh all of the points
            for (auto &item : latestPoints_)
            {
                
                auto body_id = item.first;
                auto orient = latestOrients_[body_id];
                auto point = item.second;

                // try and calculate the velocity
                double vel = 0.0;
                if (previousPoints_.find(body_id) != previousPoints_.end())
                {
                    auto prevPoint = previousPoints_[body_id];
                    
                    double dx = std::abs(point.x - prevPoint.x);
                    double dy = std::abs(point.y - prevPoint.y);
                    vel = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

                    latestVelocities_[body_id] = vel;
                }

                // update the internel costmap
                // using the center point, create a polygon
                auto polygon = constructPolygons(point, orient, vel, min_x, min_y, max_x, max_y);
                setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);
            }
            previousPoints_ = latestPoints_;
            previousOrients_ = latestOrients_;
            previousVelocities_ = latestVelocities_;
        }
        else
        {
            previousPoints_.clear();
            previousOrients_.clear();
            previousVelocities_.clear();
        }
    }
    void HumanLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        this->updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }

    void HumanLayer::activate()
    {
        onInitialize();
    }
    void HumanLayer::deactivate()
    {
        humansSub_.shutdown();
    }
    void HumanLayer::reset()
    {
        deactivate();
        //clear the whole map
        resetMaps();
        activate();
    }

    void HumanLayer::HumansCB(const tbd_ros_msgs::HumanBodyArray &msg)
    {
        if (msg.bodies.size() > 0)
        {
            geometry_msgs::TransformStamped transform;
            try
            {
                // look up a transformation
                if (ignoreTimeStamp_)
                {
                    transform = tf_->lookupTransform(operating_frame_id_, msg.bodies[0].header.frame_id, msg.bodies[0].header.stamp);
                }
                else
                {
                    transform = tf_->lookupTransform(operating_frame_id_, msg.bodies[0].header.frame_id, ros::Time(0));
                }
                // collect the data
                latestPoints_.clear();
                latestOrients_.clear();
                latestVelocities_.clear();
                for (const auto &body : msg.bodies)
                {

                    // std::cout << body.body_id << std::endl;
                    auto body_id = body.body_id;

                    for (const auto &joint : body.joints)
                    {
                        // use pelvis as the center
                        if (joint.joint_id == joint.JOINT_PELVIS)
                        {
                            auto pelvisPoint = joint.pose.position;
                            auto pelvisOrient = joint.pose.orientation; // get the orientation of the person
                            // try to transform it
                            geometry_msgs::Point transformedPoint;
                            tf2::doTransform(pelvisPoint, transformedPoint, transform);

                            // std::cout << "here" << std::endl;
                            latestPoints_[body_id] = transformedPoint;
                            latestOrients_[body_id] = pelvisOrient;
                            // std::cout << "there" << std::endl;
                            break;
                        }
                    }
                }
                lastMsgTime_ = msg.bodies[0].header.stamp;
            }
            catch (tf2::LookupException &ex)
            {
                ROS_ERROR("No Transform %s\n", ex.what());
            }
            catch (tf2::ExtrapolationException &ex)
            {
                ROS_ERROR("Extrapolation: %s\n", ex.what());
            }
        }
        else
        {
            latestPoints_.clear();
            latestOrients_.clear();
            latestVelocities_.clear();
        }
    }

} //namespace costmap_2d