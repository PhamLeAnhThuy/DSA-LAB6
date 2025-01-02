#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
using namespace std;

class Graph {
private:
    int vertices;
    vector<vector<int>> adjList;
    vector<vector<pair<int, double>>> weightedGraph; 

public:
    Graph(int v) {
        vertices = v;
        adjList.resize(v);
        weightedGraph.resize(v);
    }

    void addEdge(int src, int dest) {
        adjList[src].push_back(dest);
        adjList[dest].push_back(src); 
    }

    void addWeightedEdge(int src, int dest, double weight) {
        weightedGraph[src].push_back({dest, weight});
        weightedGraph[dest].push_back({src, weight});
    }

    // Exercise 8: Community Detection (Basic Modular Partition)
    void detectCommunities() {
        vector<int> community(vertices, -1);
        int communityId = 0;

        function<void(int, int)> dfsCommunity = [&](int node, int id) {
            community[node] = id;
            for (int neighbor : adjList[node]) {
                if (community[neighbor] == -1) {
                    dfsCommunity(neighbor, id);
                }
            }
        };

        for (int i = 0; i < vertices; i++) {
            if (community[i] == -1) {
                dfsCommunity(i, communityId++);
            }
        }

        cout << "Communities Detected:" << endl;
        for (int id = 0; id < communityId; id++) {
            cout << "Community " << id << ": ";
            for (int i = 0; i < vertices; i++) {
                if (community[i] == id) {
                    cout << i << " ";
                }
            }
            cout << endl;
        }
    }

    // Exercise 9: PageRank Algorithm
    void pageRank(double dampingFactor = 0.85, int iterations = 100) {
        vector<double> rank(vertices, 1.0 / vertices);

        for (int iter = 0; iter < iterations; iter++) {
            vector<double> newRank(vertices, (1.0 - dampingFactor) / vertices);
            for (int u = 0; u < vertices; u++) {
                for (int v : adjList[u]) {
                    newRank[v] += dampingFactor * rank[u] / adjList[u].size();
                }
            }
            rank = newRank;
        }

        cout << "PageRank Scores:" << endl;
        for (int i = 0; i < vertices; i++) {
            cout << "Node " << i << ": " << rank[i] << endl;
        }
    }

    // Exercise 10: Optimized Dijkstra's Algorithm
    void dijkstra(int src) {
        vector<double> dist(vertices, INFINITY);
        dist[src] = 0;

        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
        pq.push({0, src});

        while (!pq.empty()) {
            auto [currentDist, u] = pq.top();
            pq.pop();

            if (currentDist > dist[u]) continue;

            for (auto [v, weight] : weightedGraph[u]) {
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        cout << "Shortest distances from source " << src << ":\n";
        for (int i = 0; i < vertices; i++) {
            cout << "Node " << i << " : " << (dist[i] == INFINITY ? -1 : dist[i]) << endl;
        }
    }

    // Exercise 11: A* Algorithm for Route Planning
    void aStar(int start, int goal, vector<pair<int, int>> positions) {
        auto heuristic = [&](int u, int v) {
            return sqrt(pow(positions[u].first - positions[v].first, 2) + pow(positions[u].second - positions[v].second, 2));
        };

        vector<double> gCost(vertices, INFINITY);
        vector<double> fCost(vertices, INFINITY);
        gCost[start] = 0;
        fCost[start] = heuristic(start, goal);

        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
        pq.push({fCost[start], start});

        vector<int> cameFrom(vertices, -1);

        while (!pq.empty()) {
            auto [currentF, current] = pq.top();
            pq.pop();

            if (current == goal) {
                cout << "Path found: ";
                vector<int> path;
                while (current != -1) {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                reverse(path.begin(), path.end());
                for (int node : path) {
                    cout << node << " ";
                }
                cout << endl;
                return;
            }

            for (auto [neighbor, weight] : weightedGraph[current]) {
                double tentativeG = gCost[current] + weight;
                if (tentativeG < gCost[neighbor]) {
                    cameFrom[neighbor] = current;
                    gCost[neighbor] = tentativeG;
                    fCost[neighbor] = gCost[neighbor] + heuristic(neighbor, goal);
                    pq.push({fCost[neighbor], neighbor});
                }
            }
        }

        cout << "No path found!" << endl;
    }
};

int main() {
    Graph g(6);

    // Add edges
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.addEdge(1, 4);
    g.addEdge(2, 4);
    g.addEdge(3, 5);
    g.addEdge(4, 5);

    // Add weighted edges
    g.addWeightedEdge(0, 1, 1.0);
    g.addWeightedEdge(0, 2, 2.0);
    g.addWeightedEdge(1, 3, 1.0);
    g.addWeightedEdge(1, 4, 2.0);
    g.addWeightedEdge(2, 4, 2.5);
    g.addWeightedEdge(3, 5, 1.0);
    g.addWeightedEdge(4, 5, 1.5);

    // Positions for A* (x, y coordinates)
    vector<pair<int, int>> positions = {{0, 0}, {1, 1}, {0, 2}, {2, 1}, {1, 3}, {2, 3}};

    // Exercise 8: Community Detection
    cout << "\nExercise 8: Community Detection\n";
    g.detectCommunities();

    // Exercise 9: PageRank
    cout << "\nExercise 9: PageRank\n";
    g.pageRank();

    // Exercise 10: Optimized Dijkstra
    cout << "\nExercise 10: Dijkstra's Algorithm\n";
    g.dijkstra(0);

    // Exercise 11: Route Planning with A*
    cout << "\nExercise 11: A* Route Planning\n";
    g.aStar(0, 5, positions);

    return 0;
}