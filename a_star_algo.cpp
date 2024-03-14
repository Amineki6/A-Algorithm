#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <algorithm> 

using namespace std;

// Define the structure for a node in the graph
struct Node {
    char id;
    vector<pair<Node*, int>> adjacents; // Pair of adjacent node and cost
    int h; // Heuristic value of the node

    Node(char id, int h) : id(id), h(h) {}
};

// Define a structure to compare the cost of nodes
struct CompareCost {
    bool operator()(pair<Node*, int> const& p1, pair<Node*, int> const& p2) {
        // Compare total estimated cost (f = g + h)
        return p1.second + p1.first->h > p2.second + p2.first->h;
    }
};

// A* Algorithm function
void AStar(Node* start, Node* goal) {
    priority_queue<pair<Node*, int>, vector<pair<Node*, int>>, CompareCost> frontier;
    frontier.push(make_pair(start, 0)); // (node, cost from start)

    map<Node*, Node*> came_from;
    map<Node*, int> cost_so_far;
    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Node* current = frontier.top().first;
        frontier.pop();

        if (current == goal) {
            break;
        }

        for (auto next : current->adjacents) {
            int new_cost = cost_so_far[current] + next.second;
            if (cost_so_far.find(next.first) == cost_so_far.end() || new_cost < cost_so_far[next.first]) {
                cost_so_far[next.first] = new_cost;
                int priority = new_cost + next.first->h;
                frontier.push(make_pair(next.first, priority));
                came_from[next.first] = current;
            }
        }
    }

    // Reconstruct path (if needed)
    vector<Node*> path;
    for (Node* at = goal; at != start; at = came_from[at]) {
        path.push_back(at);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    // Display path
    cout << "Path: ";
    for (auto node : path) {
        cout << node->id << " ";
    }
    cout << endl;
}

int main() {
    // Example usage
    Node a('A', 10), b('B', 8), c('C', 5), d('D', 7), e('E', 3), f('F', 0);

    // Setting up the graph
    a.adjacents = {{&b, 4}, {&c, 1}};
    b.adjacents = {{&d, 1}};
    c.adjacents = {{&d, 5}, {&e, 3}};
    d.adjacents = {{&e, 1}};
    e.adjacents = {{&f, 2}};

    AStar(&a, &f);

    return 0;
}


