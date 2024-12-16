
#ifndef EDGE_HPP
#define EDGE_HPP


using namespace std;

class Edge {
public:
    int u;         // Extremidade u da aresta
    int v;         // Extremidade v da aresta
    double weight;    // Peso da aresta

    // Construtor da classe
    Edge(int u, int v, double weight) : u(u), v(v), weight(weight) {}
};

#endif // EDGE_HPP
