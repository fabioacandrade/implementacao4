# Min Cut s/t

## Segmentation Energy

- Ao invés de variáveis binárias que indicam se um pixel está no objeto ou não, utiliza-se uma abordagem baseada em regiões (region-based).

## Grafo G(V, E)

- *s/t graph cuts*
  - *V*: Pixels em um grafo de grid 2D.
  - Pixels vizinhos conectados por arestas n-links.
  - Depende da intensidade |I_p - I_q|.
  - "Seeds" conectadas por arestas t-links aos terminais (source e sink).

- *Max-Flow*:
  - Utiliza o algoritmo de Ford-Fulkerson para aumentar o fluxo de "água" do source (s) até o sink (t), saturando arestas suficientes para gerar um corte (cut) que separa os terminais.
  - Bottlenecks são n-links baratos.

- *Termos de energia*:
  - *Boundary term* define o custo de n-links.
  - *Regional term* define o custo de t-links.
  - Arestas com baixo custo são escolhas atrativas para o min-cost cut.

- *Hard constraints* (seeds):
  - Implementados via t-links de custo infinito.

- *Segmentação global ótima*:
  - Satisfazendo essas seeds pode ser computada em tempo polinomial de baixa ordem usando algoritmos de max-flow/min-cut.

## O Algoritmo

1. Dado um grafo G = (V, E):
   - *Vértices*: Pixels.
   - Dois vértices especiais, *S* (Source) e *T* (Sink), representam o objeto e o fundo que queremos separar.
   - Vértices vizinhos são conectados por arestas chamadas *n-links*, onde "n" equivale a neighbor.
     - Utiliza-se um sistema de 8 vizinhos (diagonais e lados).
   - *Arestas t-links* conectam pixels aos terminais.
   - Todas as arestas possuem peso não negativo.
2. O objetivo é computar o melhor/ótimo corte de S para T.
   - Utiliza o algoritmo de Boykov e Kolmogorov (2004) de graph cut.

## Energia de Segmentação

- Considere:
  - *P*: Conjunto de pixels.
  - *N*: Vizinhança de pares não ordenados {p, q} de elementos vizinhos em P.
  - *A: Vetor binário que especifica *assignments para pixels p em P, onde cada A_p pode ser "obj" (objeto) ou "bkg" (background).
  - *A* define a segmentação.

### Fórmula de energia:

E(A) = λ * R(A) + B(A)

- Onde:
  - R(A) = Σ_{p ∈ P} R_p(A_p) (Regional term).
  - B(A) = Σ_{ {p,q} ∈ N } B_{p,q} * δ(A_p ≠ A_q) (Boundary term).

δ(A_p ≠ A_q) = 
  - 1, se A_p ≠ A_q
  - 0, se A_p = A_q

- O coeficiente λ ≥ 0 especifica a importância relativa das propriedades da região R(A) em relação ao B(A).
- R(A) assume penalidades individuais para o pixel p:
  - Os custos para o "object" e "background" são dados por R_p(“obj”) e R_p(“bkg”), respectivamente.

### Exemplo:

R_p(“obj”) = - ln Pr(I_p|“obj”)

R_p(“bkg”) = - ln Pr(I_p|“bkg”)