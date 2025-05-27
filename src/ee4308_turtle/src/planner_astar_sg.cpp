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
        initParam(node_, plugin_name_ + ".max_access_cost", max_access_cost_, 150);
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
        open_list.queue(nodes.getNode(start_mx, start_my)); // push the start node into the open list

        // A* Algorithm
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
                // Backtrace the path
                for (auto p = n; p != nullptr; p = p->parent)
                {
                    std::array<int, 2> p_coord = {p->mx, p->my}; // Convert node to coordinates
                    coords.push_back(p_coord); // Push the coordinates into the vector
                }

                // Reverse the coordinates to get the path from start to goal
                std::reverse(coords.begin(), coords.end());

                // Savitzky-Golay Smoothing
                // Create a matrix for Savitzky-Golay smoothing
                int window_size = 2 * sg_half_window_ + 1;
                Eigen::MatrixXd J(window_size, sg_order_ + 1);
                for (int i = 0; i < window_size; ++i)
                {
                    for (int j = 0; j <= sg_order_; ++j)
                    {
                        J(i, j) = pow(i - sg_half_window_, j);
                    }
                }
                Eigen::MatrixXd Jt = J.transpose();
                Eigen::MatrixXd A = (Jt * J).inverse() * Jt;

                // Smoothing the path
                std::vector<std::array<int, 2>> smoothed_coords(coords.size());
                for (int i = 0; i < static_cast<int>(coords.size()); ++i)
                {
                    Eigen::VectorXd x(window_size);
                    Eigen::VectorXd y(window_size);

                    // Get the coordinates for the window
                    for (int j = -sg_half_window_; j <= sg_half_window_; ++j)
                    {
                        int idx;
                        if (i + j < 0)
                        {
                            idx = 0;
                        }
                        else if (i + j >= static_cast<int>(coords.size()))
                        {
                            idx = coords.size() - 1;
                        }
                        else
                        {
                            idx = i + j;
                        }

                        x(j + sg_half_window_) = coords[idx][0];
                        y(j + sg_half_window_) = coords[idx][1];
                    }
                    // Smooth the coordinates
                    smoothed_coords[i][0] = (A.row(0) * x).sum();
                    smoothed_coords[i][1] = (A.row(0) * y).sum();
                }

                smoothed_coords = post_processing(smoothed_coords);
                // convert smoothed coordinates to path
                return writeToPath(smoothed_coords, goal);
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
                    if (cost >= max_access_cost_) // Hardcoded value, the max_access_cost_ parameter is not working, try it out.
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
                    
                    // Calculate the cost of neighbor
                    double g = n->g + sqrt(dx * dx + dy * dy);

                    // Update the node if the cost is less than the current cost
                    if (g < m->g)
                    {
                        m->g = g;
                        m->parent = n;
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

        coords = post_processing(coords);
        return writeToPath(coords, goal);
    }

    inline bool Planner::isCellFree(const nav2_costmap_2d::Costmap2D *costmap, int mx, int my)
    {
        return costmap->getCost(mx, my) < max_access_cost_;
    }

    bool Planner::isLineOfSightClear(const std::array<int, 2> &start, const std::array<int, 2> &end) {
        RayTracer ray_tracer(start[0], start[1], end[0], end[1]);
        while (!ray_tracer.reached())
        {
            auto cell = ray_tracer.frontCell();
            if (!isCellFree(costmap_, cell[0] + 0.5, cell[1] + 0.5))
            {
                return false;
            }
            ray_tracer.next();
        }
        return true;
    }

    std::vector<std::array<int, 2>> Planner::combineCollinear(const std::vector<std::array<int, 2>> &coords) {
        if (coords.size() < 3) {
            return coords;
        }
        std::vector<std::array<int, 2>> result;
        result.push_back(coords.front());
        for (size_type i = 1; i < coords.size() - 1; ++i) {
            int x0 = result.back()[0];
            int y0 = result.back()[1];
            int x1 = coords[i][0];
            int y1 = coords[i][1];
            int x2 = coords[i + 1][0];
            int y2 = coords[i + 1][1];
            // Vector1 = (x1 - x0, y1 - y0)
            // Vector2 = (x2 - x1, y2 - y1)
            int vx1 = x1 - x0;
            int vy1 = y1 - y0;
            int vx2 = x2 - x1;
            int vy2 = y2 - y1;
            // Two vectors (vx1, vy1) and (vx2, vy2) are collinear if cross product = 0
            // cross = vx1*vy2 - vy1*vx2
            int cross = vx1 * vy2 - vy1 * vx2;
            if (cross == 0) {
              // (i) is collinear, so skip adding coords[i].
              // We effectively "merge" it. Just continue.
              continue;
            } else {
              // Not collinear. Keep coords[i].
              result.push_back(coords[i]);
            }
        }
        // Always add the last point
        result.push_back(coords.back());
        return result;
    }

    std::vector<std::array<int, 2>> Planner::replaceRightAngleWithDiagonal(
        const std::vector<std::array<int, 2>> &coords) {
        if (coords.size() < 3) {
            return coords;
        }
        std::vector<std::array<int, 2>> result;
        result.push_back(coords.front());
        size_type i = 0;
        while (i < coords.size() - 2)
        {
            const auto &A = coords[i];
            const auto &B = coords[i + 1];
            const auto &C = coords[i + 2];
            int Ax = A[0], Ay = A[1];
            int Bx = B[0], By = B[1];
            int Cx = C[0], Cy = C[1];
            int ABx = Bx - Ax;
            int ABy = By - Ay;
            int BCx = Cx - Bx;
            int BCy = Cy - By;
            // Check if AB and BC are perpendicular and of length 1
            // i.e. (|ABx|+|ABy|==1 or maybe 1 step in 8 directions)
            // and (|BCx|+|BCy|==1).
            // Then see if diagonal from A->C is only 1 cell diagonal.
            // e.g. if ABx=1, ABy=0, BCx=0, BCy=1 => A->C would be (1,1) offset => diagonal
            bool ab_is_unit = (std::abs(ABx) + std::abs(ABy) == 1);
            bool bc_is_unit = (std::abs(BCx) + std::abs(BCy) == 1);
            // Dot product zero => perpendicular, or we can check combinations
            bool perpendicular = (ABx * BCx + ABy * BCy) == 0;
            // The diagonal from A->C is presumably (Cx - Ax, Cy - Ay). This should be
            // either (1,1), (1,-1), (-1,1), or (-1,-1) for a small corner.
            bool diagonal_candidate = (std::abs(Cx - Ax) == 1 && std::abs(Cy - Ay) == 1);
            
            if (ab_is_unit && bc_is_unit && perpendicular && diagonal_candidate)
            {
                // Check if A->C is free of obstacles
                if (isLineOfSightClear(A, C))
                {
                    // We can merge B out by going directly diagonal A->C
                    result.push_back(C);
                    i += 2; // skip point B and move on
                    continue;
                }
            }
            // Otherwise keep the next point and move forward
            result.push_back(coords[i + 1]);
            i++;
        }
        // Add any remaining points
        while (i < coords.size()) {
          result.push_back(coords[i]);
          i++;
        }
        return result;
    }

    std::vector<std::array<int, 2>> Planner::mergeWithLineOfSight(const std::vector<std::array<int, 2>> &coords) {
        if (coords.size() < 2) {
            return coords;
        }
        std::vector<std::array<int, 2>> result;
        result.reserve(coords.size());
        size_type i = 0;
        result.push_back(coords[0]);
        while (i < coords.size() - 1)
        {
            size_type next = i + 1;
            // Try to jump as far ahead as possible with clear line of sight
            while (next < coords.size() - 1 && isLineOfSightClear(coords[i], coords[next + 1]))
            {
              next++;
            }
            // next is the furthest we can go in a straight line from i
            result.push_back(coords[next]);
            i = next;
        }
        return result;
    }

    std::vector<std::array<int, 2>> Planner::post_processing(const std::vector<std::array<int, 2>> &coords) {
        if (coords.size() < 2)
        {
            return coords;
        }
        // // Combine collinear points
        // auto collinear = combineCollinear(coords);
        // // Replace right angles with diagonal
        // auto diagonal = replaceRightAngleWithDiagonal(collinear);
        // // Merge with line of sight
        // auto merged = mergeWithLineOfSight(diagonal);
        // debug: merge directly with line of sight
        auto merged = mergeWithLineOfSight(coords);
        return merged;
    }

    nav_msgs::msg::Path Planner::writeToPath(
        std::vector<std::array<int, 2>> coords,
        geometry_msgs::msg::PoseStamped goal)
    {
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = global_frame_id_;
        path.header.stamp = node_->now();

        for (const auto &coord : coords)
        { 
            // convert map coordinates to world coordiantes
            double wx, wy;
            costmap_->mapToWorld(coord[0], coord[1], wx, wy);
            
            // push the pose into the messages.
            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information. 
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);
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