//
// Created by mohamad on 11.07.22.
//
#include "dijkstra.hpp"
#include <vector>
#include <iostream>
#include <chrono>
using namespace std::chrono;
using namespace std;

int main() {
    int sx = -5;
    int sy = -5;
    int gx = 50;
    int gy = 50;
    int grid_size = 2;
    float robot_radius = 1.0;

    vector<float> ox;
    vector<float> oy;
    for (int i = -10; i < 60; ++i) {
        ox.push_back((float)i);
        oy.push_back(-10);
    }
    for (int i = -10; i < 60; ++i) {
        ox.push_back(60);
        oy.push_back((float)i);
    }
    for (int i = -10; i < 61; ++i) {
        ox.push_back(1);
        oy.push_back(60);
    }
    for (int i = -10; i < 61; ++i) {
        ox.push_back(-10);
        oy.push_back((float)i);
    }
    for (int i = -10; i < 40; ++i) {
        ox.push_back(20);
        oy.push_back((float)i);
    }
    for (int i = 0; i < 40; ++i) {
        ox.push_back(40);
        oy.push_back(60 - (float)i);
    }


    auto start = high_resolution_clock::now();
    auto dijkstra = Dijkstra(ox, oy, grid_size, robot_radius);
    auto result = dijkstra.planning(sx, sy, gx, gy);
    auto duration = duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "finished in: " << duration.count() << " milliseconds" << std::endl;
    cout << "rx, ty: " << endl;
    for (int i = 0 ; i < result.first.size(); ++i)
    {
        cout << result.first[i] << ", " << result.second[i] << endl;
    }

    return 0;
}
