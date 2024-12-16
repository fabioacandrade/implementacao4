#include "edge.hpp"
#include <algorithm>
#include <iostream>
#include <vector>
using namespace std;

bool compareEdges(const Edge &e1, const Edge &e2)
{
  return e1.weight < e2.weight; // Ordena em ordem crescente de peso
}

class Graph
{
public:
  vector<vector<pair<int, double>>>
      adjacency_list; // Lista de adjacência: cada vértice tem uma lista de
                      // pares (vértice vizinho, peso)
  int size;

  Graph(int size)
  {
    this->size = size;
    adjacency_list.resize(size); // Inicializa o vetor de listas
  }

  void sort_edges(vector<Edge> &edges)
  {
    sort(edges.begin(), edges.end(),
         compareEdges); // Ordena as arestas pelo peso
  }
  // Função para adicionar uma aresta
  void add_edge(int u, int v, double weight)
  {
    adjacency_list[u].emplace_back(v, weight);
  }

  // Função para obter todas as arestas do grafo
  vector<Edge> get_edges()
  {
    vector<Edge> edge_list;
    for (int u = 0; u < size; ++u)
    {

      for (int i = 0; i < adjacency_list[u].size(); ++i)
      {
        int v = adjacency_list[u][i].first;          // Vértice vizinho
        double weight = adjacency_list[u][i].second; // Peso da aresta
                                                     // Evitar repetição de arestas
        if (u < v)
        {

          edge_list.push_back(Edge(u, v, weight));
        }
      }
    }
    sort_edges(edge_list);
    return edge_list;
  }
};
;
