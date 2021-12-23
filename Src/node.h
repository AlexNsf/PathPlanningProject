#ifndef NODE_H
#define NODE_H

#include <tuple>

//That's the data structure for storing a single search node.
//You MUST store all the intermediate computations occuring during the search
//incapsulated to Nodes (so NO separate arrays of g-values etc.)

struct Node
{
    int     i, j; //grid cell coordinates
    double  F, g, H; //f-, g- and h-values of the search node
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)

    Node(int i_, int j_, double g_, double H_, Node* parent_) : i(i_), j(j_), g(g_), H(H_), parent(parent_) {
        F = g + H;
    }
};


struct NodeComparator {
    bool operator()(const Node* node_1, const Node* node_2) const {
        return std::tie(node_1->F, node_1->i, node_1->j) < std::tie(node_2->F, node_2->i, node_2->j);
    }
};

#endif
