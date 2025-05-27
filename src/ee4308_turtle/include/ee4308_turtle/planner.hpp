
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "eigen3/Eigen/Dense"

#include "ee4308_core/core.hpp"
#include "ee4308_core/raytracer.hpp"

#pragma once

namespace ee4308::turtle
{
    // ====================== Planner Node ===================
    struct PlannerNode
    {
        double f = INFINITY;
        double g = INFINITY;
        double h = INFINITY;
        PlannerNode *parent = nullptr;
        int mx = -1; 
        int my = -1; 
        bool expanded = false;

        PlannerNode(int mx, int my);
    };

    // ======================= Open List ===========================
    struct OpenListComparator
    {
        bool operator()(PlannerNode *l, PlannerNode *r) const;
    };

    class OpenList
    {
    private:
        std::priority_queue<PlannerNode *, std::deque<PlannerNode *>, OpenListComparator> pq;

    public:
        void queue(PlannerNode *node);
        PlannerNode *pop();
        bool empty() const;
    };

    // ======================== Nodes ===============================
    class PlannerNodes
    {
    private:
        std::vector<PlannerNode> nodes;
        int size_mx, size_my;

    public:
        PlannerNodes(int num_cells_x, int num_cells_y);
        PlannerNode *getNode(int mx, int my);
    };

    // ======================== Nav2 Planner Plugin ===============================
    class Planner : public nav2_core::GlobalPlanner
    {
    public:
        Planner() = default;
        ~Planner() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

    protected:
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D *costmap_; // global costmap
        std::string global_frame_id_;
        typedef std::vector<std::array<int, 2>>::size_type size_type;

        // parameters
        int max_access_cost_;
        double interpolation_distance_;
        int sg_half_window_;
        int sg_order_;
        
        nav_msgs::msg::Path writeToPath(std::vector<std::array<int, 2>> coords, geometry_msgs::msg::PoseStamped goal);

        inline bool isCellFree(const nav2_costmap_2d::Costmap2D *costmap, int mx, int my);
        bool isLineOfSightClear(const std::array<int, 2> &start, const std::array<int, 2> &end);
        std::vector<std::array<int, 2>> combineCollinear(const std::vector<std::array<int, 2>> &coords);
        std::vector<std::array<int, 2>> replaceRightAngleWithDiagonal(const std::vector<std::array<int, 2>> &coords);
        std::vector<std::array<int, 2>> mergeWithLineOfSight(const std::vector<std::array<int, 2>> &coords);
        std::vector<std::array<int, 2>> post_processing(const std::vector<std::array<int, 2>> &coords);
    };

}