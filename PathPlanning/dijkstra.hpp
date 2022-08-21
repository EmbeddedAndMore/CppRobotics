#ifndef DIJKSTRA
#define DIJKSTRA

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

typedef std::pair<float, float> Point;

class Node {
  public:
    float x;
    float y;
    float cost;
    int parent_index;

    Node(float x, float y, float cost, int parent_idx)
        : x(x), y(y), cost(cost), parent_index(parent_idx) {}
};

class Dijkstra {

  private:
    Point min_point; // smallest point in the list of obstacles
    Point max_point; // biggest point in the list of obstacles

    float x_width; // width of the scene in x-axis
    float y_width; // width of the scene in y-axis

    std::vector<std::vector<bool>> obstacle_map; // map of obstacle in the scene

    float resolution;   // Resolution of the scene (e.g. 2 means every 2 number
                      // creates a cell)
    float robot_radius; // Radius of the robot in the scene

    std::array<std::array<float, 3>, 8>
        motion; // step and cost of movement in each direction.

    inline float idx2pos(float index, float min_val) const {
        return (index * resolution) + min_val;
    }

    inline float pos2idx(float pos, float min_val) const {
        return std::trunc((pos - min_val) / resolution);
    }

    inline float calc_index(Node &node) const {
        return (node.y - min_point.second) * x_width +
               (node.x - min_point.first);
    }

    inline static float euclidean_dist(float p1, float p2) {
        return float(std::sqrt(std::pow(p1, 2) + std::pow(p2, 2)));
    }

    bool verify_node(Node& node) const {
        float px = idx2pos(node.x, min_point.first);
        float py = idx2pos(node.y, min_point.second);

        if (px < min_point.first)
            return false;
        if (py < min_point.second)
            return false;
        if (px >= max_point.first)
            return false;
        if (py >= max_point.second)
            return false;

        if (obstacle_map[int(node.x)][int(node.y)])
            return false;

        return true;
    }

    static bool cost_lt(std::pair<int, std::shared_ptr<Node>> &&left,
                        std::pair<int, std::shared_ptr<Node>> &&right) {
        return left.second->cost < right.second->cost;
    }

  public:
    Dijkstra(std::vector<float> ox, std::vector<float> oy, float resolution,
             float robot_radius)
        : resolution(resolution), robot_radius(robot_radius) {
        motion = get_motion_model();
        calc_obstacle_map(ox, oy);
    }

    std::array<std::array<float, 3>, 8> get_motion_model() {
        // dx, dy, cost
        return {{{1, 0, 1},
                 {0, 1, 1},
                 {-1, 0, 1},
                 {0, -1, 1},
                 {1, 1, float(std::sqrt(2))},
                 {1, -1, float(std::sqrt(2))},
                 {-1, 1, float(std::sqrt(2))},
                 {-1, -1, float(std::sqrt(2))}}};
    }

    void calc_obstacle_map(std::vector<float> &ox, std::vector<float> &oy) {
        min_point = Point(std::round(*std::min_element(ox.begin(), ox.end())),
                          std::round(*std::min_element(oy.begin(), oy.end())));
        max_point = Point(std::round(*std::max_element(ox.begin(), ox.end())),
                          std::round(*std::max_element(ox.begin(), ox.end())));


        x_width =
            std::round((max_point.first - min_point.first) / resolution);
        y_width =
            std::round((max_point.second - min_point.second) / resolution);


        // obstacle map generation
        for (int i = 0; i < int(x_width); ++i) {
            std::vector<bool> y_obs;
            for (int j = 0; j < int(x_width); ++j) {
                y_obs.push_back(false);
            }
            obstacle_map.push_back(y_obs);
        }

        for (int ix = 0; ix < int(x_width); ++ix) {
            float x = idx2pos(float(ix), min_point.first);
            for (int iy = 0; iy < int(y_width); ++iy) {
                float y = idx2pos(float(iy), min_point.second);
                for (int i = 0; i < ox.size(); ++i) {
                    float iox = ox[i];
                    float ioy = oy[i];
                    float d = euclidean_dist(iox - x, ioy - y);
                    if (d <= robot_radius) {
                        obstacle_map[ix][iy] = true;
                        break;
                    }
                }
            }
        }
    }

    std::pair<std::vector<float>, std::vector<float>>
    calc_final_path(Node &goal_node,
                    std::map<int, std::shared_ptr<Node>> closed_set) {
        std::vector<float> rx = {idx2pos(goal_node.x, min_point.first)};
        std::vector<float> ry = {idx2pos(goal_node.y, min_point.second)};

        int parent_index = goal_node.parent_index;
        while (parent_index != -1) {
            auto n = closed_set[parent_index];
            rx.push_back(idx2pos(n->x, min_point.first));
            ry.push_back(idx2pos(n->y, min_point.second));
            parent_index = n->parent_index;
        }
        return {rx, ry};
    }

    std::pair<std::vector<float>, std::vector<float>> planning(float sx, float sy,
                                                               float gx, float gy) {
        Node start_node = Node(pos2idx(sx, min_point.first),
                               pos2idx(sy, min_point.second), 0.0, -1);
        Node goal_node = Node(pos2idx(gx, min_point.first),
                              pos2idx(gy, min_point.second), 0.0, -1);


        std::map<int, std::shared_ptr<Node>> open_set;
        std::map<int, std::shared_ptr<Node>> closed_set;

        open_set[(int)calc_index(start_node)] = std::make_shared<Node>(start_node);

        while (true) {
            auto c_id = *std::min_element(open_set.begin(), open_set.end(),
                                          &Dijkstra::cost_lt);
            Node current = *c_id.second;

            if (current.x == goal_node.x && current.y == goal_node.y) {
                goal_node.parent_index = current.parent_index;
                goal_node.cost = current.cost;
                break;
            }

            // Remove the item from the open set
            open_set.erase(c_id.first);
            // Add it to the closed set
            closed_set[c_id.first] = std::make_shared<Node>(current);

            // expand search grid based on motion model
            for (auto &item : motion) {
                auto node =
                    Node(current.x + item[0], current.y + item[1],
                         current.cost + item[2], c_id.first);
                int n_id = (int)calc_index(node);
                if (closed_set.find(n_id) != closed_set.end())
                    continue;

                if (!verify_node(node))
                    continue;

                if (open_set.find(n_id) == open_set.end())
                    open_set[n_id] = std::make_shared<Node>(node);
                else {
                    if (open_set[n_id]->cost >= node.cost) {
                        // This path is the best until now. record it!
                        open_set[n_id] = std::make_shared<Node>(node);

                    }
                }
            }
        }
        auto rxy = calc_final_path(goal_node, closed_set);
        return rxy;
    }
};

#endif