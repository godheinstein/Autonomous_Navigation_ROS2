#include "ee4308_turtle/planner.hpp"

namespace ee4308::turtle
{

    // ======================== Nav2 Planner Plugin ===============================
    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        tf_ = tf;
        plugin_name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros->getGlobalFrameID();

        // initialize parameters
        initParam(node_, plugin_name_ + ".max_access_cost", max_access_cost_, 175);
        initParam(node_, plugin_name_ + ".interpolation_distance", interpolation_distance_, 0.05);
        initParam(node_, plugin_name_ + ".sg_half_window", sg_half_window_, 5);
        initParam(node_, plugin_name_ + ".sg_order", sg_order_, 3);
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        // initializations
        PlannerNodes nodes(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
        OpenList open_list;
        RayTracer ray_tracer;

        int start_mx, start_my, goal_mx, goal_my;
        costmap_->worldToMapEnforceBounds(
            start.pose.position.x, start.pose.position.y,
            start_mx, start_my);
        costmap_->worldToMapEnforceBounds(
            goal.pose.position.x, goal.pose.position.y,
            goal_mx, goal_my);

        // Create vector to store new path
        std::vector<std::array<int, 2>> coords;

        // Set the size of the costmap, eases further calculations
        int size_x = static_cast<int>(costmap_->getSizeInCellsX());
        int size_y = static_cast<int>(costmap_->getSizeInCellsY());

        // Iterate through all nodes and set g to infinity and parent to nullptr
        for (int x = 0; x < size_x; ++x)
        {
            for (int y = 0; y < size_y; ++y)
            {
                nodes.getNode(x, y)->g = std::numeric_limits<double>::infinity();
                nodes.getNode(x, y)->parent = nullptr;
            }
        }

        // Initialize start node
        nodes.getNode(start_mx, start_my)->g = 0; // initialize start node with g = 0;
        nodes.getNode(start_mx, start_my)->expanded = true; // initialize start node as expanded;
        open_list.queue(nodes.getNode(start_mx, start_my)); // push the start node into the open list

        // Theta* Algorithm
        // Initialize all neighbors
        auto start_n = nodes.getNode(start_mx, start_my);
        for (int dx_s = -1; dx_s <= 1; ++dx_s)
        {
            for (int dy_s = -1; dy_s <= 1; ++dy_s)
            {
                // Ignore the node if it is the same as the current node
                if (dx_s == 0 && dy_s == 0)
                {
                    continue;
                }
                int mx = start_n->mx + dx_s;
                int my = start_n->my + dy_s;

                // check if the node is within the map
                if (mx < 0 || my < 0 || mx >= size_x || my >= size_y)
                {
                    continue;
                }

                // Get neighbor node
                PlannerNode *m = nodes.getNode(mx, my);

                // Check if the node is an obstacle through thresholding
                auto cost = costmap_->getCost(mx, my);
                if (cost >= max_access_cost_)
                {
                    continue;
                }

                // Check if node is null
                if (m == nullptr)
                {
                    continue;
                }
                
                // Calculate the cost of neighbor
                double g = start_n->g + sqrt(dx_s * dx_s + dy_s * dy_s);

                // Update the node if the cost is less than the current cost(always will be)
                if (g < m->g)
                {
                    m->g = g;
                    m->parent = start_n;
                    m->f = g + sqrt((mx - goal_mx) * (mx - goal_mx) + (my - goal_my) * (my - goal_my));
                    open_list.queue(m);
                }
            }
        }


        // Iterate through the open list
        while (!open_list.empty())
        {
            // Getting the node with the lowest f value
            auto n = open_list.pop();
            
            // Ignore if the node is already expanded
            if (n->expanded)
            {
                continue;
            }

            // If the node is the goal node, then we have found the path
            if (n->mx == goal_mx && n->my == goal_my)
            {
                // std::vector<PlannerNode *> pnodes;
                for (auto p = n; p != nullptr; p = p->parent)
                {
                    coords.push_back({p->mx, p->my});
                }

                // coords.push_back({start_mx, start_my});
                std::reverse(coords.begin(), coords.end());

                // convert coordinates to path
                return writeToPath(coords, goal);
            }

            // Mark the node as expanded
            n->expanded = true;

            // Iterate through all the neighbors of the node
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    // Ignore the node if it is the same as the current node
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }

                    int mx = n->mx + dx;
                    int my = n->my + dy;

                    // check if the node is within the map
                    if (mx < 0 || my < 0 || mx >= size_x || my >= size_y)
                    {
                        continue;
                    }

                    // Get neighbor node
                    PlannerNode *m = nodes.getNode(mx, my);

                    // Check if the node is an obstacle through thresholding
                    auto cost = costmap_->getCost(mx, my);
                    if (cost >= max_access_cost_)
                    {
                        continue;
                    }

                    // Check if node is null
                    if (m == nullptr)
                    {
                        continue;
                    }

                    // Check if expanded
                    if (m->expanded)
                    {
                        continue;
                    }

                    // Defining parent of n 
                    PlannerNode *n_ancient = n -> parent;
                    ee4308::RayTracer raytracer = RayTracer(mx, my, n_ancient -> mx, n_ancient -> my);

                    // Check if parent is within sight
                    while (raytracer.reached() == false)
                    {   
                        PlannerNode *los = nodes.getNode(raytracer.frontCell()[0] + 0.5, raytracer.frontCell()[1] + 0.5);

                        if (costmap_->getCost(los -> mx, los -> my) > max_access_cost_)
                        {
                            n_ancient = n;
                            break;
                        }   
                        raytracer.next();
                    }
                    
                    // Calculate the cost of neighbor
                    double g = n_ancient -> g + sqrt((n_ancient -> mx - mx) * (n_ancient -> mx - mx) + (n_ancient -> my - my) * (n_ancient -> my - my));

                    // Update the node if the cost is less than the current cost
                    if (g < m->g)
                    {
                        m->g = g;
                        m->parent = n_ancient;
                        m->f = g + sqrt((mx - goal_mx) * (mx - goal_mx) + (my - goal_my) * (my - goal_my));
                        open_list.queue(m);
                    }
                }
            }
        }
        
        // check if path is empty
        if (coords.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "No path found from start to goal");
        }
        return writeToPath(coords, goal);
    }

    nav_msgs::msg::Path Planner::writeToPath(
        std::vector<std::array<int, 2>> coords,
        geometry_msgs::msg::PoseStamped goal)
    {
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = global_frame_id_;
        path.header.stamp = node_->now();

        std::vector<std::array<double, 2>> wcoords;
        for (const auto &coord : coords)
        { 
            // convert map coordinates to world coordiantes
            double wx, wy;
            costmap_->mapToWorld(coord[0], coord[1], wx, wy);

            wcoords.push_back({wx, wy});
        }

        for (int i = 0; i < (int)wcoords.size() - 1; ++i)
        {
            auto prev_coords = wcoords[i];
            auto next_coords = wcoords[i+1];

            double dx = next_coords[0] - prev_coords[0];
            double dy = next_coords[1] - prev_coords[1];
            double dist = std::hypot(dx, dy);

            double intp = 0.05;
            double d = 0;
            while (d < dist)
            {
                double x = prev_coords[0] + d / dist * dx;
                double y = prev_coords[1] + d / dist * dy;

                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.orientation.w = 1;
                path.poses.push_back(pose);

                d += intp;  
            }
        }

        // push the goal
        goal.header.frame_id = "";  // remove frame id to prevent incorrect transformations.
        goal.header.stamp = rclcpp::Time();  // remove timestamp from header, otherwise there will be time extrapolation issues.
        path.poses.push_back(goal);

        // return path;
        return path;
    }


    void Planner::cleanup()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::activate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::deactivate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }


    // ====================== Planner Node ===================
    PlannerNode::PlannerNode(int mx, int my) : mx(mx), my(my) {}

    // ======================= Open List Implemetations ===========
    bool OpenListComparator::operator()(PlannerNode *l, PlannerNode *r) const { return l->f > r->f; }

    void OpenList::queue(PlannerNode *node) { pq.push(node); }

    PlannerNode *OpenList::pop()
    {
        if (pq.empty())
            return nullptr;
        PlannerNode *cheapest_node = pq.top();
        pq.pop();
        return cheapest_node;
    }

    bool OpenList::empty() const { return pq.empty(); }

    // ======================== Nodes ===============================
    PlannerNodes::PlannerNodes(int num_cells_x, int num_cells_y)
    {
        size_mx = num_cells_x;
        size_my = num_cells_y;

        nodes.reserve(num_cells_x * num_cells_y);
        for (int mx = 0; mx < size_mx; ++mx)
            for (int my = 0; my < size_my; ++my)
                nodes[mx * size_my + my] = PlannerNode(mx, my);
    }

    PlannerNode *PlannerNodes::getNode(int mx, int my)
    {
        if (mx < 0 || my < 0 || mx >= size_mx || my >= size_my)
            return nullptr;
        return &nodes[mx * size_my + my];
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)