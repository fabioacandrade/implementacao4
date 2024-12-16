
#include "edge.hpp"
#include <iostream>
#include <unordered_map>
#include <vector>

using namespace std;

class UnionFind
{
  int size;
  vector<int> parent, rank;

public:
  vector<int> Int, component_size;
  UnionFind(int size)
  {
    this->size = size;
    Int.resize(size, 0);
    parent.resize(size);
    rank.resize(size, 0);
    component_size.resize(size, 1);
    for (int i = 0; i < size; i++)
    {
      parent[i] = i;
    }
  }

  int find(int x)
  {
    if (parent[x] != x)
    {
      // Path compression
      parent[x] = find(parent[x]);
    }
    return parent[x];
  }

  void union_by_rank(int x, int y, Edge current)
  {
    int rootX = find(x);
    int rootY = find(y);

    if (rootX != rootY)
    { // Apenas une se forem componentes diferentes
      if (rank[rootX] > rank[rootY])
      {
        parent[rootY] = rootX;
        component_size[rootX] += component_size[rootY];
        Int[rootX] = current.weight;
      }
      else if (rank[rootX] < rank[rootY])
      {
        parent[rootX] = rootY;
        component_size[rootY] += component_size[rootX];
        Int[rootY] = current.weight;
      }
      else
      { // Mesmo rank: escolha arbitrária e incremente o rank
        parent[rootY] = rootX;
        component_size[rootX] += component_size[rootY];
        Int[rootX] = current.weight;
        rank[rootX]++;
      }
    }
  }
};
