
# Algoritmo de Segmentação de Imagens Baseado em Grafos

## Descrição do Problema

Considere um grafo não direcionado **G(V, E)** ponderado, onde:
- **V**: conjunto de vértices (pixels).
- **E**: conjunto de arestas entre os vértices.
- **w(e)**: peso da aresta **e**, medindo a dissimilaridade entre os vértices conectados.

A **segmentação** é definida como a divisão do conjunto **V** em **componentes conexos**.

---

### Definições Importantes

1. **Internal Difference (Int(C)):**  
   A maior aresta da Árvore Geradora Mínima (MST) do componente conexo **C**:  
   `Int(C) = max w(e), para e ∈ MST(C)`

2. **Difference Between (Dif(C1, C2)):**  
   O menor peso de aresta conectando os componentes conexos **C1** e **C2**:  
   `Dif(C1, C2) = min w((vi, vj)), para vi ∈ C1, vj ∈ C2, e (vi, vj) ∈ E`  
   - Se não existir uma aresta entre **C1** e **C2**, então `Dif(C1, C2) = infinito`.

3. **Critério de Diferença (D(C1, C2)):**  
   Determina se os componentes devem ser unidos:
   - `D(C1, C2) = true` se `Dif(C1, C2) > MInt(C1, C2)`
   - `D(C1, C2) = false` caso contrário.

4. **Minimum Internal Difference (MInt(C1, C2)):**  
   Define o limite para união de componentes:
   `MInt(C1, C2) = min(Int(C1) + t(C1), Int(C2) + t(C2))`

5. **Função de Controle (t(C)):**  
   Controla o grau de diferença necessário para unir dois componentes:  
   `t(C) = k / |C|`  
   - **k**: constante definida previamente.
   - **|C|**: número de vértices no componente **C**.  
     - Quanto maior **k**, menos divisões (menos componentes).
     - Quanto menor **k**, mais divisões (mais componentes).

---

## Algoritmo de Segmentação

**Entrada:**  
- Um grafo **G = (V, E)** com **n** vértices e **m** arestas.

**Saída:**  
- **S = {C1, C2, C3, ..., Cr}**: componentes conexos do grafo.

### Passos

1. **Ordenar as arestas:**  
   Ordenar o conjunto **E** em ordem não-decrescente de peso **w(e)**.

2. **Inicializar componentes:**  
   Cada vértice **v ∈ V** forma inicialmente um componente conexo isolado.

3. **Processar arestas:**  
   Para cada aresta **eq** (q = 1, 2, ..., m) na ordem:
   - Considere **eq = (vi, vj)**, conectando os vértices **vi** e **vj**.
   - Determine os componentes **C1** e **C2** que contêm **vi** e **vj**, respectivamente.
   - Se **C1 ≠ C2** e **w(eq) < MInt(C1, C2)**:
     - Unir **C1** e **C2** em um único componente.
   - Caso contrário, manter os componentes separados.

4. **Finalizar:**  
   Retornar **S = Sm**, a partição final do grafo em componentes conexos.

---

## Considerações

- A constante **k** ajusta a granularidade da segmentação.
- O algoritmo é eficiente com a ordenação inicial e a utilização de estruturas para manipular componentes (como Union-Find).
- Já garante uma AGM por seguir a lógica do Kruskall.


