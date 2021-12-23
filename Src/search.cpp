#include "search.h"
#include <memory>

std::string CoordinatesToString(int i, int j) {
    return std::to_string(i) + ' ' + std::to_string(j);
}

Search::Search()
{
//set defaults here
}

Search::~Search() {
    for (const auto& node : visited_nodes) {
        delete node.second;
    }
}

double Search::CalculateHeuristic(int i, int j, int goal_i, int goal_j, const EnvironmentOptions &options) {
    int delta_x = abs(goal_i - i);
    int delta_y = abs(goal_j - j);
    switch (options.metrictype) {
        case CN_SP_MT_DIAG:
            return std::min(delta_x, delta_y) * (CN_SQRT_TWO - 1) + std::max(delta_x, delta_y);
        case CN_SP_MT_MANH:
            return delta_x + delta_y;
        case CN_SP_MT_EUCL:
            return sqrt(delta_x * delta_x + delta_y * delta_y);
        case CN_SP_MT_CHEB:
            return std::max(delta_x, delta_y);
        default:
            return 0;
    }
}

void Search::AddNeighboursToOpen(Node *node, const Map &map, const EnvironmentOptions &options) {
    for (int i = -1; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
            if (i == 0 && j == 0) {
                continue;
            }
            double len_add = 1.0;
            int new_i = node->i + i;
            int new_j = node->j + j;
            if (!map.CellOnGrid(new_i, new_j) || !map.CellIsTraversable(new_i, new_j)) {
                continue;
            }
            if (abs(i) == abs(j)) {
                len_add = CN_SQRT_TWO;
                if (!options.allowdiagonal) {
                    continue;
                }
                if ((map.CellIsObstacle(node->i, node->j + j) || map.CellIsObstacle(node->i + i, node->j)) && !options.cutcorners) {
                    continue;
                }
                if (map.CellIsObstacle(node->i, node->j + j) && map.CellIsObstacle(node->i + i, node->j + j) && !options.allowsqueeze) {
                    continue;
                }
            }
            std::string coordinates_string = CoordinatesToString(new_i, new_j);
            if (visited_nodes[coordinates_string]) {
                if (CLOSE.find(coordinates_string) != CLOSE.end()) {
                    continue;
                }
                double g_gap = visited_nodes[coordinates_string]->g - (node->g + len_add);
                if (g_gap > 0) {
                    visited_nodes[coordinates_string]->F -= g_gap;
                    visited_nodes[coordinates_string]->g -= g_gap;
                    visited_nodes[coordinates_string]->parent = node;
                    if (OPEN.find(visited_nodes[coordinates_string]) != OPEN.end()) {
                        OPEN.erase(visited_nodes[coordinates_string]);
                    }
                    OPEN.insert(visited_nodes[coordinates_string]);
                }
            } else {
                std::pair<int, int> goal_coordinates = map.getGoal();
                visited_nodes[coordinates_string] = new Node(new_i, new_j, node->g + len_add,
                                                         CalculateHeuristic(new_i, new_j, goal_coordinates.first,
                                                                            goal_coordinates.second, options), node);
                OPEN.insert(visited_nodes[coordinates_string]);
            }
        }
    }
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::pair<int, int> start_coordinates = map.getStart();
    std::pair<int, int> goal_coordinates = map.getGoal();
    Node* goal = nullptr;
    Node* start = new Node{start_coordinates.first, start_coordinates.second, 0,
                           CalculateHeuristic(start_coordinates.first, start_coordinates.second,
                                              goal_coordinates.first, goal_coordinates.second, options), nullptr};
//    std::cout << start->H << '\n';
    sresult.numberofsteps = 1;
    OPEN.insert(start);
    while (true) {
        ++sresult.numberofsteps;
        if (OPEN.empty()) {
            sresult.pathfound = false;
            break;
        }
        Node* cur_node = *OPEN.begin();
        OPEN.erase(OPEN.begin());
        if (cur_node->i == goal_coordinates.first && cur_node->j == goal_coordinates.second) {
            sresult.pathfound = true;
            goal = cur_node;
            break;
        }
        AddNeighboursToOpen(cur_node, map, options);

        CLOSE[CoordinatesToString(cur_node->i, cur_node->j)] = cur_node;
    }

    if (sresult.pathfound) {
        sresult.pathlength = goal->g;
        makePrimaryPath(goal);
    } else {
        sresult.pathlength = 0;
    }
    sresult.lppath = &lppath;
    sresult.hppath = &hppath;
    auto t2 = std::chrono::high_resolution_clock::now();
    sresult.time = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    sresult.nodescreated = visited_nodes.size();

//    std::cout << sresult.numberofsteps << " STEPS\n";
//    std::cout << sresult.nodescreated << " NODES\n";
//    std::cout << sresult.pathlength << " PATHLEN\n";

    /*sresult.pathfound = ;
    sresult.nodescreated =  ;
    sresult.numberofsteps = ;
    sresult.time = ;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;*/
    return sresult;
}

void Search::makePrimaryPath(Node* curNode) {
    while (curNode) {
        lppath.push_front(*curNode);
        curNode = curNode->parent;
    }
}

/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
