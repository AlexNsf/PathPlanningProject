#include "search.h"
#include <memory>

Search::Search()
{
//set defaults here
}

Search::~Search() {}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    std::pair<int, int> start_coordinates = map.getStart();
    std::pair<int, int> goal_coordinates = map.getGoal();
    Node* goal = nullptr;
    Node* start = new Node{start_coordinates.first, start_coordinates.second, 0, 0, 0, nullptr};
    sresult.nodescreated = 1;
    sresult.numberofsteps = 1;
    OPEN.insert({0, start});
    while (true) {
        if (OPEN.empty()) {
            sresult.pathfound = false;
            break;
        }
        Node* cur_node = OPEN.begin()->second;
        OPEN.erase(OPEN.begin());
        if (cur_node->i == goal_coordinates.first && cur_node->j == goal_coordinates.second) {
            sresult.pathfound = true;
            goal = cur_node;
            break;
        }
        for (int i = -1; i < 2; ++i) {
            for (int j = -1; j < 2; ++j) {
                if (!((i == 0 && j == 0) || (i != 0 && j != 0))) {
                    ++sresult.numberofsteps;
                    bool flag = false;
                    if (map.CellOnGrid(cur_node->i + i, cur_node->j + j) && map.CellIsTraversable(cur_node->i + i, cur_node->j + j)) {
                        for (Node* closed_node : CLOSE) {
                            if (closed_node->i == cur_node->i + i && closed_node->j == cur_node->j + j) {
                                flag = true;
                                break;
                            }
                        }
                        if (!flag) {
                            for (auto dist_and_opened_node : OPEN) {
                                Node* opened_node = dist_and_opened_node.second;
                                if (opened_node->i == cur_node->i + i && opened_node->j == cur_node->j + j) {
                                    flag = true;
                                    if (opened_node->g > cur_node->g + map.getCellSize()) {
                                        opened_node->g = cur_node->g + map.getCellSize();
                                    }
                                }
                            }
                            if (!flag) {
                                Node* new_opened_node = new Node{cur_node->i + i, cur_node->j + j, cur_node->g + map.getCellSize(), cur_node->g + map.getCellSize(), 0, cur_node};
                                ++sresult.nodescreated;
                                OPEN.insert({new_opened_node->g, new_opened_node});
                            }
                        }
                    }
                }
            }
        }

    }

    if (sresult.pathfound) {
        sresult.pathlength = goal->g;
    } else {
        sresult.pathlength = 0;
    }
    sresult.lppath = &lppath;
    sresult.hppath = &hppath;
    auto t2 = std::chrono::high_resolution_clock::now();
    sresult.time = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

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
