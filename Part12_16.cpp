#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <cmath>
using namespace std;

class Graph {
private:
    int vertices;
    vector<vector<int>> adjList;
    unordered_map<int, vector<int>> connections; 
    vector<vector<int>> grid; 

public:
    Graph(int v) {
        vertices = v;
        adjList.resize(v);
        grid.resize(v, vector<int>(v, 0));
    }

    void addEdge(int src, int dest) {
        adjList[src].push_back(dest);
        adjList[dest].push_back(src); 
        connections[src].push_back(dest);
        connections[dest].push_back(src);
    }

    // Exercise 12: Find Influential Users in Social Network
    void findInfluentialUsers() {
        vector<int> degrees(vertices, 0);
        for (int i = 0; i < vertices; i++) {
            degrees[i] = adjList[i].size();
        }
        int maxDegree = *max_element(degrees.begin(), degrees.end());
        cout << "Most influential users (high degree): ";
        for (int i = 0; i < vertices; i++) {
            if (degrees[i] == maxDegree) {
                cout << i << " ";
            }
        }
        cout << endl;
    }

    // Exercise 13: Identify Traffic Bottlenecks
    void identifyTrafficBottlenecks() {
        vector<int> bottlenecks(vertices, 0);
        for (int i = 0; i < vertices; i++) {
            for (auto neighbor : adjList[i]) {
                bottlenecks[i]++;
            }
        }
        cout << "Traffic bottlenecks (high edge connections): ";
        for (int i = 0; i < vertices; i++) {
            if (bottlenecks[i] > 2) { 
                cout << i << " ";
            }
        }
        cout << endl;
    }

    // Exercise 14: Recommendation System
    void recommendProducts(int user) {
        set<int> recommended;
        for (auto friendNode : connections[user]) {
            for (auto item : connections[friendNode]) {
                if (item != user && connections[user].end() == find(connections[user].begin(), connections[user].end(), item)) {
                    recommended.insert(item);
                }
            }
        }
        cout << "Recommended products for user " << user << ": ";
        for (auto product : recommended) {
            cout << product << " ";
        }
        cout << endl;
    }

    // Exercise 15: Optimize Network Topology
    void optimizeNetworkTopology() {
        cout << "Optimized network topology using Minimum Spanning Tree:\n";
        vector<pair<int, pair<int, int>>> edges;
        for (int i = 0; i < vertices; i++) {
            for (int j = i + 1; j < vertices; j++) {
                if (grid[i][j] > 0) {
                    edges.push_back({grid[i][j], {i, j}});
                }
            }
        }
        sort(edges.begin(), edges.end());
        vector<int> parent(vertices);
        for (int i = 0; i < vertices; i++) parent[i] = i;

        auto find = [&](int u) {
            while (u != parent[u]) u = parent[u];
            return u;
        };

        for (auto [w, nodes] : edges) {
            int src = find(nodes.first);
            int dest = find(nodes.second);
            if (src != dest) {
                parent[src] = dest;
                cout << nodes.first << " - " << nodes.second << " (Weight: " << w << ")\n";
            }
        }
    }

    // Exercise 16: NPC Pathfinding with A* Algorithm
    struct Node {
        int x, y, cost, heuristic;
        bool operator<(const Node& other) const {
            return (cost + heuristic) > (other.cost + other.heuristic);
        }
    };

    void aStarPathfinding(int startX, int startY, int goalX, int goalY) {
        priority_queue<Node> pq;
        vector<vector<bool>> visited(vertices, vector<bool>(vertices, false));
        pq.push({startX, startY, 0, abs(goalX - startX) + abs(goalY - startY)});

        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();

            if (current.x == goalX && current.y == goalY) {
                cout << "Path found with cost: " << current.cost << endl;
                return;
            }

            if (visited[current.x][current.y]) continue;
            visited[current.x][current.y] = true;

            vector<pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
            for (auto [dx, dy] : directions) {
                int nx = current.x + dx, ny = current.y + dy;
                if (nx >= 0 && ny >= 0 && nx < vertices && ny < vertices && !visited[nx][ny]) {
                    pq.push({nx, ny, current.cost + 1, abs(goalX - nx) + abs(goalY - ny)});
                }
            }
        }
        cout << "Path not found!" << endl;
    }
};

int main() {
    Graph g(5);

    // Add edges
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 4);

    // Social network analysis
    cout << "Exercise 12:\n";
    g.findInfluentialUsers();

    // Traffic bottlenecks
    cout << "\nExercise 13:\n";
    g.identifyTrafficBottlenecks();

    // Recommendation system
    cout << "\nExercise 14:\n";
    g.recommendProducts(0);

    // Network optimization
    cout << "\nExercise 15:\n";
    g.optimizeNetworkTopology();

    // NPC Pathfinding
    cout << "\nExercise 16:\n";
    g.aStarPathfinding(0, 0, 4, 4);

    return 0;
}
