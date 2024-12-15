#include "edge.hpp"
#include <algorithm>
#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

class Graph
{
public:
  struct Node
  {
    int parent;               // Nodo pai no crescimento da árvore
    double residual_capacity; // Capacidade residual
    bool in_tree;             // Indica se está na árvore de busca
  };

  vector<vector<pair<int, double>>> adjacency_list; // Lista de adjacência
  int size;

  Graph(int size)
  {
    this->size = size;
    adjacency_list.resize(size);
  }

  void add_edge(int u, int v, double weight)
  {
    adjacency_list[u].emplace_back(v, weight);
    adjacency_list[v].emplace_back(u, 0); // Adiciona uma aresta reversa com capacidade 0
  }

  // Algoritmo de Boykov-Kolmogorov
  double boykov_kolmogorov(int source, int sink)
  {
    vector<Node> nodes(size);
    for (int i = 0; i < size; ++i)
    {
      nodes[i].parent = -1;
      nodes[i].residual_capacity = 0;
      nodes[i].in_tree = false;
    }

    queue<int> active;
    active.push(source);
    nodes[source].in_tree = true;
    nodes[source].residual_capacity = numeric_limits<double>::infinity();

    // Fase de crescimento
    while (!active.empty())
    {
      int u = active.front();
      active.pop();

      cout << "Processando no: " << u << endl; // Log de depuração

      for (auto &[v, capacity] : adjacency_list[u])
      {
        if (!nodes[v].in_tree && capacity > 0)
        { // Aresta não saturada
          nodes[v].parent = u;
          nodes[v].residual_capacity = min(nodes[u].residual_capacity, capacity);
          nodes[v].in_tree = true;

          if (v == sink)
          { // Caminho aumentado encontrado
            // Fase de atualização
            double flow = nodes[v].residual_capacity;
            cout << "Caminho aumentado encontrado!" << endl;
            for (int curr = sink; curr != source; curr = nodes[curr].parent)
            {
              int prev = nodes[curr].parent;

              // Atualizar capacidades residuais
              for (auto &edge : adjacency_list[prev])
              {
                if (edge.first == curr)
                {
                  edge.second -= flow; // Diminui a capacidade da aresta
                  break;
                }
              }
              for (auto &edge : adjacency_list[curr])
              {
                if (edge.first == prev)
                {
                  edge.second += flow; // Aumenta a capacidade reversa
                  break;
                }
              }
            }

            return flow; // Retorna o fluxo encontrado
          }

          active.push(v);
        }
      }
    }

    return 0; // Nenhum fluxo adicional possível
  }

  vector<bool> min_cut(int source, int sink)
  {
    double max_flow = 0;

    while (true)
    {
      double flow = boykov_kolmogorov(source, sink);
      if (flow == 0)
        break; // Não há mais caminhos aumentados
      max_flow += flow;
    }

    // Determinar os vértices alcançáveis a partir do source
    vector<bool> visited(size, false);
    queue<int> q;
    q.push(source);
    visited[source] = true;

    while (!q.empty())
    {
      int u = q.front();
      q.pop();

      for (auto &[v, capacity] : adjacency_list[u])
      {
        if (capacity > 0 && !visited[v])
        { // Aresta não saturada
          visited[v] = true;
          q.push(v);
        }
      }
    }

    return visited; // Retorna os vértices no lado do source
  }
};