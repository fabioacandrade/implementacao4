#include "graph.cpp"
#include "seeds.cpp"
#include "struct.hpp"
#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;

double LAMBDA;
double SIGMA = 0.0;


void calculate_lambda(const Image &image) {
  double mean_intensity = 0.0;
  double variance = 0.0;
  int total_pixels = image.width * image.height;

  // Calcular a média das intensidades
  for (int i = 0; i < image.height; i++) {
    for (int j = 0; j < image.width; j++) {
      mean_intensity +=
          (image.pixel_matrix[i][j].R + image.pixel_matrix[i][j].G +
           image.pixel_matrix[i][j].B) /
          3.0;
    }
  }
  mean_intensity /= total_pixels;

  // Calcular a variância das intensidades
  for (int i = 0; i < image.height; i++) {
    for (int j = 0; j < image.width; j++) {
      double intensity =
          (image.pixel_matrix[i][j].R + image.pixel_matrix[i][j].G +
           image.pixel_matrix[i][j].B) /
          3.0;
      variance += pow(intensity - mean_intensity, 2);
    }
  }
  variance /= total_pixels;

  // Definir λ como uma função da variância
  double lambda = variance / (mean_intensity + 1e-6); // Evitar divisão por zero
  LAMBDA = lambda > 0 ? lambda : 1.0; // Garantir que λ seja positivo
}
double calculate_K(const Image &image) {
  double max_sum = 0.0;

  // Percorre todos os pixels da imagem
  for (int i = 0; i < image.height; i++) {
    for (int j = 0; j < image.width; j++) {
      double sum = 0.0;

      // Conecta o pixel atual aos seus vizinhos
      for (int di = -1; di <= 1; di++) {
        for (int dj = -1; dj <= 1; dj++) {
          if (abs(di) + abs(dj) == 1) { // Apenas vizinhos
            int ni = i + di;
            int nj = j + dj;
            if (ni >= 0 && ni < image.height && nj >= 0 && nj < image.width) {
              // Calcula a penalidade da borda
              double intensity_diff = abs(image.pixel_matrix[i][j].R -
                                          image.pixel_matrix[ni][nj].R);
              sum +=
                  exp(-pow(intensity_diff, 2) /
                      (2 * SIGMA * SIGMA)); // Usando a função de penalidade Bpq
            }
          }
        }
      }

      // Atualiza o máximo
      if (sum > max_sum) {
        max_sum = sum;
      }
    }
  }

  // Retorna K como 1 mais o máximo das somas
  return 1 + max_sum; // K é definido como 1 + max da soma das penalidades das
                      // arestas <sup>12</sup>.
}
void calculate_sigma(const Image &image) {
  // Inicializa uma variável para armazenar a soma das diferenças de intensidade
  double sum_diff = 0.0;
  int count = 0;

  // Percorre todos os pixels da imagem
  for (int i = 0; i < image.height; i++) {
    for (int j = 0; j < image.width; j++) {
      // Compara o pixel atual com seus vizinhos
      for (int di = -1; di <= 1; di++) {
        for (int dj = -1; dj <= 1; dj++) {
          if (abs(di) + abs(dj) == 1) { // Apenas vizinhos
            int ni = i + di;
            int nj = j + dj;
            if (ni >= 0 && ni < image.height && nj >= 0 && nj < image.width) {
              // Calcula a diferença de intensidade
              double diff = abs(image.pixel_matrix[i][j].R -
                                image.pixel_matrix[ni][nj].R);
              sum_diff += diff;
              count++;
            }
          }
        }
      }
    }
  }

  // Calcula a média das diferenças
  double mean_diff = sum_diff / count;

  // Estima σ como uma fração da média das diferenças
  SIGMA = mean_diff / 2.0; // Ajuste conforme necessário
}

// Função para ler o arquivo PPM
struct Image *read_file(string filename) {
  struct Image *image = new struct Image;
  string magic_number;
  ifstream file(filename, ios::binary);

  if (!file.is_open()) {
    cerr << "Erro ao abrir o arquivo: " << filename << endl;
    return nullptr;
  }

  file >> magic_number;
  if (magic_number != "P3" && magic_number != "P6") {
    cerr << "Formato do arquivo não suportado: " << magic_number << endl;
    return nullptr;
  }

  file >> image->width >> image->height >> image->max_color;
  file.ignore();

  image->pixel_matrix.resize(image->height, vector<Pixel>(image->width));

  if (magic_number == "P3") {
    for (int i = 0; i < image->height; i++) {
      for (int j = 0; j < image->width; j++) {
        file >> image->pixel_matrix[i][j].R >> image->pixel_matrix[i][j].G >>
            image->pixel_matrix[i][j].B;
        image->pixel_matrix[i][j].x = i;
        image->pixel_matrix[i][j].y = j;
      }
    }
  } else if (magic_number == "P6") {
    for (int i = 0; i < image->height; i++) {
      for (int j = 0; j < image->width; j++) {
        unsigned char rgb[3];
        file.read(reinterpret_cast<char *>(rgb), 3);
        image->pixel_matrix[i][j].R = rgb[0];
        image->pixel_matrix[i][j].G = rgb[1];
        image->pixel_matrix[i][j].B = rgb[2];
        image->pixel_matrix[i][j].x = i;
        image->pixel_matrix[i][j].y = j;
      }
    }
  }

  file.close();
  return image;
}


double intensity(Pixel p) {
  // Fórmula de luminância para a conversão de RGB para escala de cinza
  return 0.299 * p.R + 0.587 * p.G + 0.114 * p.B;
}

double dist(Pixel p1, Pixel p2) {
  // Distância espacial entre os pixels (diferença nas coordenadas)
  double spatial_distance = abs(p1.x - p2.x) + abs(p1.y - p2.y);

  // Distância de intensidade (diferença entre os valores de cor)
  double intensity_distance =
      sqrt(pow(p1.R - p2.R, 2) + pow(p1.G - p2.G, 2) + pow(p1.B - p2.B, 2));

  // Peso combinado das duas distâncias (espaço e intensidade)
  double combined_distance = spatial_distance + intensity_distance;

  // Evitar divisão por zero
  return combined_distance == 0 ? 1e-6 : combined_distance;
}
double Bpq(Pixel p1, Pixel p2, double sigma) {
  double intensity_diff = pow(intensity(p1) - intensity(p2), 2);
  double distance = dist(p1, p2);

  return exp(-intensity_diff / (2 * sigma * sigma)) * (1.0 / distance);
}

double Rp(int intensity, vector<double> &histogram) {
  double probability = histogram[intensity];
  if (probability == 0) {
    probability = 1e-6; // Usar um valor pequeno para evitar log(0)
  }
  double ln_x = -std::log(probability);

  return ln_x;
}

double weight_calc(int lambda, int intensity, vector<double> histogram) {
  double rp = Rp(intensity, histogram);
  double t = lambda * rp;
  return t;
}

// Função para converter a imagem em um grafo para Graph Cuts
Graph convert_image_to_graph_graphcuts(Image image, vector<int> labels,
                                       vector<double> histogramObj,
                                       vector<double> histogramBkg) {
  int num_vertices = image.height * image.width;
  int source = num_vertices;   // Vértice source
  int sink = num_vertices + 1; // Vértice sink

  Graph g(num_vertices + 2); // Adiciona dois vértices extras para source e sink

  // vector<double> histogramObj = build_histograms(image);

  for (int i = 0; i < image.height; i++) {
    for (int j = 0; j < image.width; j++) {
      int current_vertex = i * image.width + j;
      Pixel current_pixel = image.pixel_matrix[i][j];
      if (current_vertex != source || current_vertex != sink) {

        double to_source;
        double to_sink;
        // Conectar o pixel ao source e ao sink
        if (labels[current_vertex] == 0) {
          to_source =
              calculate_K(image); // Background, alta penalidade para o source
          to_sink = 0;            // Conexão forte com o sink
        } else if (labels[current_vertex] == 1) {
          to_source = 0; // Conexão forte com o source
          to_sink =
              calculate_K(image); // Foreground, alta penalidade para o sink
        } else {
          to_source = weight_calc(LAMBDA, intensity(current_pixel),
                                  histogramBkg); // Cálculos personalizados
          to_sink = weight_calc(LAMBDA, intensity(current_pixel), histogramObj);
        }

        g.add_edge_source(source, current_vertex,
                          to_source);                   // Fonte para pixel
        g.add_edge_sink(current_vertex, sink, to_sink); // Pixel para sink

        // Conectar o pixel aos seus vizinhos
        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            if (abs(di) + abs(dj) != 1)
              continue;

            int ni = i + di, nj = j + dj;
            if (ni >= 0 && ni < image.height && nj >= 0 && nj < image.width) {
              int neighbor_vertex = ni * image.width + nj;
              Pixel neighbor_pixel = image.pixel_matrix[ni][nj];

              double weight = Bpq(current_pixel, neighbor_pixel, 3);

              bool edge_exists = false;
              for (auto &[vertex, capacity] :
                   g.adjacency_list[current_vertex]) {
                if (vertex == neighbor_vertex) {
                  edge_exists = true;
                  break;
                }
              }

              if (!edge_exists) {
                g.add_edge(current_vertex, neighbor_vertex, weight);
              }
            }
          }
        }
      }
    }
  }
  return g;
}

vector<vector<int>> segment_image_graphcuts(Graph &g, int width, int height) {
  int source = g.size - 2;
  int sink = g.size - 1;

  // Aplicar corte mínimo
  vector<bool> is_foreground = g.min_cut(source, sink);

  // Criar matriz binária de segmentos
  vector<vector<int>> segments(height, vector<int>(width, 0));
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int vertex = i * width + j;
      segments[i][j] = is_foreground[vertex] ? 1 : 0;
    }
  }

  return segments;
}

// Salvar imagem segmentada
void saveSegmentedImage(const string &outputFilename,
                        vector<vector<int>> &segments, Image &image) {
  ofstream outFile(outputFilename + ".ppm", ios::binary);
  if (!outFile.is_open()) {
    cerr << "Erro ao abrir o arquivo: " << outputFilename + ".ppm" << endl;
    return;
  }

  // Escrever cabeçalho PPM
  outFile << "P3\n";
  outFile << image.width << " " << image.height << "\n";
  outFile << "255\n";

  // Escrever dados dos pixels
  for (int i = 0; i < image.height; i++) {
    for (int j = 0; j < image.width; j++) {
      int segment = segments[i][j];
      if (segment == 1) {
        outFile << "255 0 0 "; // Vermelho para foreground
      } else {
        outFile << "255 255 255 "; // Preto para background
      }
    }
    outFile << "\n";
  }

  outFile.close();
}

int main() {

  Image *image = read_file(filename);

  if (image == nullptr) {
    return -1;
  }

  // Determinar as seeds do objeto e do background
  vector<int> labels;
  vector<double> histogramA, histogramB;
  build_histograms(histogramA, histogramB, labels,
                   image->width * image->height);

  // Converter imagem para grafo
  Graph g =
      convert_image_to_graph_graphcuts(*image, labels, histogramA, histogramB);
  // Segmentar a imagem
  vector<vector<int>> segments =
      segment_image_graphcuts(g, image->width, image->height);

  // Salvar imagem segmentada em PPM
  saveSegmentedImage("../images/output_segmented", segments, *image);

  delete image;
  return 0;
}
