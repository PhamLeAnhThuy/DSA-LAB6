#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <climits>
#include <algorithm>
using namespace std;

class Graph {
private:
    int vertices;
    vector<vector<int>> adjList;         
    vector<vector<int>> adjMatrix;      
    vector<vector<pair<int, int>>> weightedGraph; 

public:
    Graph(int v) {
        vertices = v;
        adjList.resize(v);
        adjMatrix.resize(v, vector<int>(v, 0));
        weightedGraph.resize(v);
    }

    // Exercise 1: Add edges
    void addEdgeList(int src, int dest) {
        adjList[src].push_back(dest);
        adjList[dest].push_back(src); 
    }

    void addEdgeMatrix(int src, int dest) {
        adjMatrix[src][dest] = 1;
        adjMatrix[dest][src] = 1; 
    }

    void addWeightedEdge(int src, int dest, int weight) {
        weightedGraph[src].push_back({dest, weight});
        weightedGraph[dest].push_back({src, weight}); 
    }

    void displayList() {
        cout << "Adjacency List:" << endl;
        for (int i = 0; i < vertices; i++) {
            cout << i << ": ";
            for (auto neighbor : adjList[i]) {
                cout << neighbor << " ";
            }
            cout << endl;
        }
    }

    void displayMatrix() {
        cout << "Adjacency Matrix:" << endl;
        for (int i = 0; i < vertices; i++) {
            for (int j = 0; j < vertices; j++) {
                cout << adjMatrix[i][j] << " ";
            }
            cout << endl;
        }
    }

    // Exercise 2: DFS (Recursive & Iterative)
    void dfsRecursive(int v, vector<bool>& visited) {
        visited[v] = true;
        cout << v << " ";
        for (int neighbor : adjList[v]) {
            if (!visited[neighbor]) {
                dfsRecursive(neighbor, visited);
            }
        }
    }

    void dfsIterative(int start) {
        vector<bool> visited(vertices, false);
        stack<int> s;
        s.push(start);

        while (!s.empty()) {
            int current = s.top();
            s.pop();

            if (!visited[current]) {
                visited[current] = true;
                cout << current << " ";
                for (auto neighbor = adjList[current].rbegin(); neighbor != adjList[current].rend(); ++neighbor) {
                    if (!visited[*neighbor]) {
                        s.push(*neighbor);
                    }
                }
            }
        }
    }

    // Exercise 3: BFS
    void bfs(int start) {
        vector<bool> visited(vertices, false);
        queue<int> q;
        q.push(start);
        visited[start] = true;

        while (!q.empty()) {
            int current = q.front();
            q.pop();
            cout << current << " ";
            for (int neighbor : adjList[current]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
    }

    // Exercise 4: Detect Cycles
    bool detectCycleDirectedDFS(int v, vector<bool>& visited, vector<bool>& recStack) {
        visited[v] = true;
        recStack[v] = true;

        for (int neighbor : adjList[v]) {
            if (!visited[neighbor] && detectCycleDirectedDFS(neighbor, visited, recStack)) {
                return true;
            } else if (recStack[neighbor]) {
                return true;
            }
        }
        recStack[v] = false;
        return false;
    }

    bool hasCycleDirected() {
        vector<bool> visited(vertices, false);
        vector<bool> recStack(vertices, false);
        for (int i = 0; i < vertices; i++) {
            if (!visited[i]) {
                if (detectCycleDirectedDFS(i, visited, recStack)) {
                    return true;
                }
            }
        }
        return false;
    }

    // Exercise 5: Dijkstra's Algorithm
    void dijkstra(int src) {
        vector<int> dist(vertices, INT_MAX);
        dist[src] = 0;

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto [v, weight] : weightedGraph[u]) {
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        cout << "Shortest distances from source " << src << ": ";
        for (int i = 0; i < vertices; i++) {
            cout << dist[i] << " ";
        }
        cout << endl;
    }

    // Exercise 6: Connected Components
    void findConnectedComponents() {
        vector<bool> visited(vertices, false);
        int count = 0;

        for (int i = 0; i < vertices; i++) {
            if (!visited[i]) {
                count++;
                cout << "Component " << count << ": ";
                dfsRecursive(i, visited);
                cout << endl;
            }
        }
    }

    // Exercise 7: Find Bridges (Tarjan's Algorithm)
    void findBridgesUtil(int u, vector<bool>& visited, vector<int>& disc, vector<int>& low, vector<int>& parent) {
        static int time = 0;
        visited[u] = true;
        disc[u] = low[u] = ++time;

        for (int v : adjList[u]) {
            if (!visited[v]) {
                parent[v] = u;
                findBridgesUtil(v, visited, disc, low, parent);
                low[u] = min(low[u], low[v]);

                if (low[v] > disc[u]) {
                    cout << "Bridge: " << u << " - " << v << endl;
                }
            } else if (v != parent[u]) {
                low[u] = min(low[u], disc[v]);
            }
        }
    }

    void findBridges() {
        vector<bool> visited(vertices, false);
        vector<int> disc(vertices, -1), low(vertices, -1), parent(vertices, -1);

        for (int i = 0; i < vertices; i++) {
            if (!visited[i]) {
                findBridgesUtil(i, visited, disc, low, parent);
            }
        }
    }
};

int main() {
    Graph g(5);

    // Add edges
    g.addEdgeList(0, 1);
    g.addEdgeList(1, 2);
    g.addEdgeList(2, 0);
    g.addEdgeList(1, 3);
    g.addEdgeList(3, 4);

    // Display graph
    g.displayList();
    g.displayMatrix();

    // DFS
    cout << "\nDFS Recursive: ";
    vector<bool> visited(5, false);
    g.dfsRecursive(0, visited);

    cout << "\nDFS Iterative: ";
    g.dfsIterative(0);

    // BFS
    cout << "\nBFS: ";
    g.bfs(0);

    // Cycle Detection
    cout << "\nHas Cycle (Directed): " << (g.hasCycleDirected() ? "Yes" : "No") << endl;

    // Dijkstra's Algorithm
    g.addWeightedEdge(0, 1, 2);
    g.addWeightedEdge(1, 2, 3);
    g.addWeightedEdge(0, 2, 1);
    g.addWeightedEdge(1, 3, 5);
    g.addWeightedEdge(3, 4, 2);
    cout << "\nDijkstra: ";
    g.dijkstra(0);

    // Connected Components
    cout << "\nConnected Components:\n";
    g.findConnectedComponents();

    // Bridges
    cout << "\nBridges:\n";
    g.findBridges();

    return 0;
}
