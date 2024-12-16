#include <algorithm>
#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <unordered_map>

using namespace std;

class Graph {
public:
    vector<vector<pair<int, double>>> adjacency_list; // Lista de adjacência
    unordered_map<int, unordered_map<int, double>> capacity_map; // Mapa para capacidades das arestas
    int size;

    Graph(int size) {
        this->size = size;
        adjacency_list.resize(size);
    }

    void add_edge(int u, int v, double weight) {
        adjacency_list[u].emplace_back(v, weight);
        adjacency_list[v].emplace_back(u, 0);
        capacity_map[u][v] = weight;
        capacity_map[v][u] = 0; // Capacidade reversa
    }

    void add_edge_source(int u, int v, double weight) {
        adjacency_list[u].emplace_back(v, weight);
        capacity_map[u][v] = weight;
    }

    void add_edge_sink(int u, int v, double weight) {
        adjacency_list[u].emplace_back(v, weight);
        capacity_map[u][v] = weight;
    }

    // Algoritmo Boykov-Kolmogorov para fluxo máximo
    double boykov_kolmogorov(int source, int sink) {
        double max_flow = 0;
        vector<int> dist(size, -1); // Distância dos vértices no grafo
        vector<int> parent(size, -1); // Array de pais para reconstruir caminhos
        vector<bool> in_queue(size, false); // Marcadores para saber se um vértice está na fila
        vector<double> residual_capacity(size, 0); // Capacidades residuais

        while (true) {
            // Limpa as variáveis para a próxima iteração
            fill(dist.begin(), dist.end(), -1);
            fill(parent.begin(), parent.end(), -1);
            fill(in_queue.begin(), in_queue.end(), false);
            fill(residual_capacity.begin(), residual_capacity.end(), 0);

            queue<int> q;
            dist[source] = 0;
            residual_capacity[source] = numeric_limits<double>::infinity();
            q.push(source);
            in_queue[source] = true;


            while (!q.empty()) {
                int u = q.front();
                q.pop();
                in_queue[u] = false;

                for (auto &[v, cap] : adjacency_list[u]) {
                    if (dist[v] == -1 && capacity_map[u][v] > 0) { // Aresta não saturada
                        dist[v] = dist[u] + 1;
                        parent[v] = u;
                        residual_capacity[v] = min(residual_capacity[u], capacity_map[u][v]);

                        if (v == sink) {
                            break;
                        }

                        if (!in_queue[v]) {
                            q.push(v);
                            in_queue[v] = true;
                        }
                    }
                }
            }

            // Se não encontrou um caminho aumentado, o fluxo máximo foi alcançado
            if (dist[sink] == -1) {
                break;
            }

            // Atualizar capacidades residuais ao longo do caminho
            double path_flow = residual_capacity[sink];
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                capacity_map[u][v] -= path_flow;
                capacity_map[v][u] += path_flow;

            }

            max_flow += path_flow;
        }

        cout << "Fluxo máximo encontrado: " << max_flow << "\n";
        return max_flow;
    }

    vector<bool> min_cut(int source, int sink) {
        double max_flow = boykov_kolmogorov(source, sink);

        // Determinar os vértices alcançáveis a partir do source
        vector<bool> visited(size, false);
        queue<int> q;
        q.push(source);
        visited[source] = true;

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (auto &[v, capacity] : adjacency_list[u]) {
                if (capacity_map[u][v] > 0 && !visited[v]) { // Aresta não saturada
                    visited[v] = true;
                    q.push(v);
                }
            }
        }

        cout << "Fluxo máximo: " << max_flow << endl;

        return visited; // Retorna os vértices no lado do source
    }
};
