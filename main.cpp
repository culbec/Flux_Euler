#include <iostream>
#include <fstream>
#include <stack>
#include <vector>
#include <unordered_set>
#include <climits>
#include <queue>
#include <list>

using namespace std;

class Hierholzer {
private:
    ifstream fin;
    ofstream fout;

    int V{}, E{};

    unordered_set<int> *adj;
    vector<int> circuit;

public:
    Hierholzer(const string &in, const string &out) : fin{in}, fout{out} {
        fin >> V >> E;

        this->adj = new unordered_set<int>[V];

        for (int i = 0; i < E; i++) {
            int u, v;
            fin >> u >> v;
            adj[u].insert(v);
            adj[v].insert(u);
        }
    }

    void euler(int s) {
        stack<int> S;
        S.push(s);

        while (!S.empty()) {
            int u = S.top();
            if (!adj[u].empty()) {
                int v = *adj[u].begin();

                adj[u].erase(v);
                adj[v].erase(u);
                S.push(v);

                continue;
            }

            S.pop();
            circuit.push_back(u);
        }

        circuit.pop_back();

        for (auto &i: circuit) {
            fout << i << " ";
        }
    }

    ~Hierholzer() {
        delete[] this->adj;
    }
};

class Fleury {
private:
    ifstream fin;
    ofstream fout;
    int V, E;

    vector<vector<int>> matAdj;


    void findEulerianCircuit(vector<int> L, vector<vector<int>> mat, int u) {
        for (int v = 0; v < V; v++) {
            if (mat[u][v] != 0) {
                mat[u][v] = mat[v][u] = 0;
                findEulerianCircuit(L, mat, v);
            }
        }
        L.emplace_back(u);
    }

public:
    Fleury(const string &in, string out) : fin{in}, fout{out} {
        fin >> V >> E;

        this->matAdj.resize(V);
        for (int i = 0; i < V; i++) {
            this->matAdj[i] = vector<int>(V, 0);
        }

        for (int i = 0; i < E; i++) {
            int u, v;
            fin >> u >> v;
            matAdj[u][v] = matAdj[v][u] = 1;
        }
    }

    void euler() {
        vector<int> circuit;
        findEulerianCircuit(circuit, matAdj, 0);

        for (auto &u: circuit) {
            fout << u << " ";
        }
    }

};

class FordFulkerson {
private:
    ifstream fin;
    ofstream fout;
    int V, E;

    vector<vector<int>> rezG;
    vector<int> parent;

    // for searching the shortest path from S to T
    // in the rezidual Graph
    bool BFS(vector<vector<int>> &rezg, int S, int T, vector<int> &p) {
        vector<bool> visited(V, false);

        queue<int> Q;
        Q.push(S);

        visited[S] = true;
        parent[S] = -1;

        while (!Q.empty()) {
            int u = Q.front();
            Q.pop();

            for (int v = 0; v < V; v++) {
                if (!visited[v] && rezg[u][v] > 0) {
                    if (v == T) {
                        p[v] = u;
                        return true;
                    }

                    Q.push(v);
                    p[v] = u;
                    visited[v] = true;
                }
            }
        }
        return false;
    }

public:
    FordFulkerson(const string &in, const string &out) : fin{in}, fout{out} {
        this->fin >> this->V >> this->E;

        this->rezG.resize(V);
        this->parent.resize(V);

        for (int i = 0; i < V; i++) {
            this->rezG[i] = vector<int>(V, 0);
        }

        for (int i = 0; i < E; i++) {
            int u, v, w;
            fin >> u >> v >> w;

            this->rezG[u][v] = w;
        }
    }

    int findFlow() {
        int S = 0;
        int T = this->V - 1;

        int maxFlow = 0;

        while (BFS(this->rezG, S, T, this->parent)) {
            int pathFlow = INT_MAX;
            for (int v = T; v != S; v = this->parent[v]) {
                int u = this->parent[v];
                if (this->rezG[u][v] < pathFlow) {
                    pathFlow = this->rezG[u][v];
                }
            }

            for (int v = T; v != S; v = this->parent[v]) {
                int u = this->parent[v];
                this->rezG[u][v] -= pathFlow;
                this->rezG[v][u] += pathFlow;
            }

            maxFlow += pathFlow;
        }

        fout << maxFlow;
        return maxFlow;
    }
};

class Preflux {
    struct Vertex {
        int h, excess;
    };

    struct Edge {
        int flow, capacity;
        int u, v;
    };

private:
    ifstream fin;
    ofstream fout;
    int V, E;

    vector<Vertex> vertices;
    vector<Edge> edges;


    void pushPreflow(int S) {
        vertices[S].h = vertices.size();

        for (int i = 0; i < edges.size(); i++) {
            if (edges[i].u == S) {
                edges[i].flow = edges[i].capacity;

                vertices[edges[i].v].excess = edges[i].capacity;
                vertices[edges[i].u].excess -= edges[i].capacity;

                edges.emplace_back(-edges[i].flow, 0, edges[i].v, S);
            }
        }
    }

    int overFlowVertex(vector<Vertex> &ver) {
        for (int i = 1; i < ver.size() - 1; i++) {
            if (ver[i].excess > 0) {
                return i;
            }
        }

        return -1;
    }

    void updateReverseEdgeFlow(int i, int flow) {
        int u = edges[i].v;
        int v = edges[i].u;

        for (int j = 0; j < edges.size(); j++) {
            if (edges[j].v == v && edges[j].u == u) {
                edges[j].flow -= flow;
                return;
            }
        }

        edges.emplace_back(0, flow, u, v);
    }

    bool push(int u) {
        for (int i = 0; i < edges.size(); i++) {
            if (edges[i].u == u) {
                if (edges[i].flow == edges[i].capacity) {
                    continue;
                }
                if (vertices[u].h > vertices[edges[i].v].h) {
                    int flow = min(edges[i].capacity - edges[i].flow, vertices[u].excess);

                    vertices[u].excess -= flow;
                    vertices[edges[i].v].excess += flow;
                    edges[i].flow += flow;
                    updateReverseEdgeFlow(i, flow);

                    return true;
                }
            }
        }
        return false;
    }

    void relabel(int u) {
        int minHeight = INT_MAX;

        for (int i = 0; i < edges.size(); i++) {
            if (edges[i].u == u) {
                if (edges[i].flow == edges[i].capacity) {
                    continue;
                }
                if (vertices[edges[i].v].h < minHeight) {
                    minHeight = vertices[edges[i].v].h;
                    vertices[u].h = minHeight + 1;
                }
            }
        }
    }

public:
    Preflux(const string &in, const string &out) : fin{in}, fout{out} {
        fin >> V >> E;

        for (int i = 0; i < V; i++) {
            vertices.emplace_back(0, 0);
        }

        for (int i = 0; i < E; i++) {
            int u, v, w;
            fin >> u >> v >> w;

            edges.emplace_back(0, w, u, v);
        }
    }

    int findFlow() {
        int S = 0;
        int T = this->V - 1;

        pushPreflow(S);

        while (overFlowVertex(vertices) != -1) {
            int u = overFlowVertex(vertices);
            if (!push(u)) {
                relabel(u);
            }
        }

        int maxFlow = vertices.back().excess;

        fout << maxFlow;
        return maxFlow;
    }

};

class TopologicPreflux {
    struct Vertex {
        int h, excess;
    };

    struct Edge {
        int flow, capacity;
        int u, v;
    };
private:
    ifstream fin; ofstream fout;
    int V, E;

    vector<Vertex> vertices;
    vector<Edge> edges;

    vector<int> current;

    void pushPreflow(int S) {
        vertices[S].h = vertices.size();

        for (int i = 0; i < edges.size(); i++) {
            if (edges[i].u == S) {
                edges[i].flow = edges[i].capacity;

                vertices[edges[i].v].excess = edges[i].capacity;
                vertices[edges[i].u].excess -= edges[i].capacity;

                edges.emplace_back(-edges[i].flow, 0, edges[i].v, S);
            }
        }
    }

    void relabel(int u) {
        int minHeight = INT_MAX;

        for (int i = 0; i < edges.size(); i++) {
            if (edges[i].u == u) {
                if (edges[i].flow == edges[i].capacity) {
                    continue;
                }
                if (vertices[edges[i].v].h < minHeight) {
                    minHeight = vertices[edges[i].v].h;
                    vertices[u].h = minHeight + 1;
                }
            }
        }
    }

    void updateReverseEdgeFlow(int i, int flow) {
        int u = edges[i].v;
        int v = edges[i].u;

        for (int j = 0; j < edges.size(); j++) {
            if (edges[j].v == v && edges[j].u == u) {
                edges[j].flow -= flow;
                return;
            }
        }

        edges.emplace_back(0, flow, u, v);
    }

    void pushFlow(int u, int v) {
        for(int i = 0; i < edges.size(); i++) {
            if(edges[i].u == u && edges[i].v == v) {
                int deltaFlow = min(edges[i].capacity - edges[i].flow, vertices[u].excess);

                vertices[u].excess -= deltaFlow;
                vertices[v].excess += deltaFlow;

                edges[i].flow += deltaFlow;
                updateReverseEdgeFlow(i, deltaFlow);
            }
        }
    }

    void discharge(int u) {
        while(vertices[u].excess > 0) {
            if(current[u] == V) {
                relabel(u);
                current[u] = 0;

                continue;
            }

            int v = current[u];
            bool pushed = false;

            for(int i = 0; i < edges.size() && !pushed; i++) {
                if(edges[i].u == u && edges[i].v == v) {
                    if(edges[i].capacity - edges[i].flow > 0 && vertices[u].h == vertices[v].h + 1) {
                        pushed = true;
                        pushFlow(u, v);
                    }
                }
            }

            if(!pushed) {
                current[u]++;
            }
        }
    }

public:
    TopologicPreflux(const string& in, const string& out): fin{in}, fout{out} {
        fin >> V >> E;

        for(int i = 0; i < V; i++) {
            vertices.emplace_back(0, 0);
        }

        for(int i = 0; i < E; i++) {
            int u, v, w;
            fin >> u >> v >> w;

            edges.emplace_back(0, w, u, v);
        }
    }

    int findFlow() {
        int S = 0;
        int T = this->V - 1;

        pushPreflow(S);

        list<int> L;

        for(int i = 0; i < vertices.size(); i++) {
            if(i != S && i != T) {
                L.push_back(i);
                current[i] = 0;
            }
        }

        auto vertex = L.begin();
        while(vertex != L.end()) {
            int u = *vertex;
            int oldHeight = vertices[u].h;

            discharge(u);

            if(vertices[u].h > oldHeight) {
                L.splice(L.begin, L, vertex);
            }

            vertex++;
        }

        int maxFlow = vertices[T].excess;
        fout << maxFlow;
        return maxFlow;
    }
};

int main(int argc, char **argv) {
    /*Hierholzer hierholzer{argv[1], argv[2]};
    hierholzer.euler(1);

    Fleury fleury{argv[1], argv[2]};
    fleury.euler();*/

    /*FordFulkerson fordFulkerson{argv[1], argv[2]};
    fordFulkerson.findFlow();*/

    /*Preflux preflux{"1-in.txt", "1-out.txt"};
    preflux.findFlow();*/

    TopologicPreflux topologicPreflux{argv[1], argv[2]};
    topologicPreflux.findFlow();

    return 0;
}
