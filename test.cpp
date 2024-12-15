#include "graph.cpp"
#include "union_find.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <opencv2/opencv.hpp>
#define LAMBDA 1.0 // Peso para o termo regional

using namespace std;
using namespace cv;

const int OBJECT = 1;
const int BACKGROUND = 0;

struct Pixel {
    int x;
    int y;
    int R;
    int G;
    int B;
};

struct Image {
    int width;
    int height;
    int max_color;
    vector<vector<struct Pixel>> pixel_matrix;
};

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
                file >> image->pixel_matrix[i][j].R >> image->pixel_matrix[i][j].G >> image->pixel_matrix[i][j].B;
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

// Função para calcular a energia de borda entre dois pixels
double boundary_energy(Pixel p1, Pixel p2) {
    return sqrt(pow(p1.R - p2.R, 2) + pow(p1.G - p2.G, 2) + pow(p1.B - p2.B, 2));
}

// Função para calcular a energia regional
double regional_energy(Pixel p, int intensity_mean) {
    int intensity = (p.R + p.G + p.B) / 3;
    return abs(intensity - intensity_mean);
}

double intensity(Pixel p) {
    return (p.R + p.G + p.B) / 3.0;
}

// Função para calcular a probabilidade a priori
double prior_probability(int intensity, const std::vector<double>& histogram) {
    // A probabilidade é baseada na distribuição do histograma
    // Retorna a probabilidade correspondente à intensidade
    return histogram[intensity];
}

// Função para calcular a penalidade de borda
double boundary_penalty(Pixel p1, Pixel p2) {
    return boundary_energy(p1, p2);
}

// // Função para calcular a energia de segmentação
// double calculate_energy(const Image &image, const vector<int> &labels, double lambda) {
//     double energy = 0.0;

//     // Calcular o termo regional R(A)
//     for (int p = 0; p < image.width * image.height; ++p) {
//         if (labels[p] == OBJECT) {
//             energy += -log(prior_probability(image.pixel_matrix[p], "obj")); // R_p("obj")
//         } else {
//             energy += -log(prior_probability(image.pixel_matrix[p], "bkg")); // R_p("bkg")
//         }
//     }

//     // Calcular o termo de borda B(A)
//     for (const auto &edge : edges) {
//         int p = edge.first;
//         int q = edge.second;
//         if (labels[p] != labels[q]) {
//             energy += boundary_penalty(image.pixel_matrix[p], image.pixel_matrix[q]); // B_p,q
//         }
//     }

//     return lambda * energy + boundary_term; // E(A) = λR(A) + B(A)
// }


// Define the boundary energy function
double boundary_energy(const Pixel &p, const Pixel &q, double sigma) {
    double intensity_diff = intensity(p) - intensity(q);
    double distance = sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2)); // Calculate Euclidean distance
    return exp(- (intensity_diff * intensity_diff) / (2 * sigma * sigma)) / distance;
}

void build_histograms(const Image &image, const std::vector<int> &labels, std::vector<double> &histogram_obj, std::vector<double> &histogram_bkg) {
    // Inicializa os histogramas
    histogram_obj.resize(256, 0);
    histogram_bkg.resize(256, 0);

    // Contar as intensidades
    for (int i = 0; i < image.height; i++) {
        for (int j = 0; j < image.width; j++) {
            Pixel pixel = image.pixel_matrix[i][j];
            int intensity = (pixel.R + pixel.G + pixel.B) / 3; // Intensidade média

            if (labels[i * image.width + j] == OBJECT) {
                histogram_obj[intensity]++;
            } else {
                histogram_bkg[intensity]++;
            }
        }
    }

    // Normalizar os histogramas
    double total_obj = 0.0, total_bkg = 0.0;
    for (double value : histogram_obj) total_obj += value;
    for (double value : histogram_bkg) total_bkg += value;

    for (size_t i = 0; i < histogram_obj.size(); ++i) {
        histogram_obj[i] /= total_obj; // Normaliza o histograma do objeto
        histogram_bkg[i] /= total_bkg; // Normaliza o histograma do fundo
    }
}

// Rp(“obj”) = − ln Pr(Ip|“obj”) (5)
//Rp(“bkg”) = − ln Pr(Ip|“bkg”) (6)
// double regional_energy(const Pixel &p) {
//     int intensity = (p.R + p.G + p.B) / 3;
//     double energy_obj = -log(prior_probability(intensity, "obj"));
//     double energy_bkg = -log(prior_probability(intensity, "bkg"));
//     return energy_obj + energy_bkg;
// }


// Função para converter a imagem em um grafo para Graph Cuts
Graph convert_image_to_graph_graphcuts(Image &image, int intensity_mean, double sigma) {
    int num_vertices = image.height * image.width;
    int source = num_vertices;      // Vértice source
    int sink = num_vertices + 1;   // Vértice sink

    Graph g(num_vertices + 2); // Adiciona dois vértices extras para source e sink

    // vector<double> histogramObj = build_histograms(image);

    for (int i = 0; i < image.height; i++) {
        for (int j = 0; j < image.width; j++) {
            int current_vertex = i * image.width + j;
            Pixel current_pixel = image.pixel_matrix[i][j];

            // Conectar o pixel ao source e ao sink
            double to_source = regional_energy(current_pixel, intensity_mean);
            double to_sink = regional_energy(current_pixel, 255 - intensity_mean);
            cout << "Fonte para pixel (" << i << "," << j << "): " << to_source << ", Sink: " << to_sink << endl;

            g.add_edge(source, current_vertex, to_source); // Fonte para pixel
            g.add_edge(current_vertex, sink, to_sink);     // Pixel para sink

            // Conectar o pixel aos seus vizinhos
            for (int di = -1; di <= 1; ++di) {
                for (int dj = -1; dj <= 1; ++dj) {
                    if (abs(di) + abs(dj) != 1) continue;

                    int ni = i + di, nj = j + dj;
                    if (ni >= 0 && ni < image.height && nj >= 0 && nj < image.width) {
                        int neighbor_vertex = ni * image.width + nj;
                        Pixel neighbor_pixel = image.pixel_matrix[ni][nj];

                        double weight = boundary_energy(current_pixel, neighbor_pixel, sigma);
                        g.add_edge(current_vertex, neighbor_vertex, weight);
                    }
                }
            }
        }
    }
    return g;
}

// Algoritmo para realizar Graph Cuts
vector<vector<int>> segment_image_graphcuts(Graph &g, int width, int height) {
    int num_vertices = width * height;
    int source = num_vertices;
    int sink = num_vertices + 1;

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
void saveSegmentedImage(const string &outputFilename, vector<vector<int>> &segments, Image &image) {
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
                outFile << "0 0 0 "; // Preto para background
            }
        }
        outFile << "\n";
    }

    outFile.close();
}

// Função de callback para capturar eventos do mouse
void onMouse(int event, int x, int y, int, void*) {
    if (event == EVENT_LBUTTONDOWN) { // Clique esquerdo -> seeds do objeto
        objectSeeds.emplace_back(x, y);
        circle(displayImage, Point(x, y), 5, Scalar(0, 255, 0), -1); // Marca com círculo verde
        cout << "Objeto seed: (" << x << ", " << y << ")\n";
    } else if (event == EVENT_RBUTTONDOWN) { // Clique direito -> seeds de fundo
        backgroundSeeds.emplace_back(x, y);
        circle(displayImage, Point(x, y), 5, Scalar(0, 0, 255), -1); // Marca com círculo vermelho
        cout << "Background seed: (" << x << ", " << y << ")\n";
    }
    imshow("Seleção de Pixels", displayImage); // Atualizar a imagem exibida
}

// Função para capturar as seeds do objeto e do fundo
void capturarSeeds(Mat& image) {
    displayImage = image.clone();  // Criar uma cópia para exibir com os marcadores

    // Criar janela e associar callback de mouse
    namedWindow("Seleção de Pixels", WINDOW_AUTOSIZE);
    setMouseCallback("Seleção de Pixels", onMouse); // Usando a função de callback global

    cout << "Clique com o botão esquerdo para definir seeds do objeto (verde).\n";
    cout << "Clique com o botão direito para definir seeds do fundo (vermelho).\n";
    cout << "Pressione 'ESC' para finalizar a seleção.\n";

    // Loop principal para exibir a imagem
    while (true) {
        imshow("Seleção de Pixels", displayImage);
        if (waitKey(30) == 27) break; // Pressione 'ESC' para sair
    }
}

int main() {
    string filename = "C:/workspace/grafos/implementacao4/baseball.ppm";

     Image *image = read_file(filename);

     if (image == nullptr) {
        return -1;
    }

    // Determinar as seeds do objeto e do background
    capturarSeeds(filename, objectSeeds, backgroundSeeds);

    // Converter imagem para grafo
    // Graph g = convert_image_to_graph_graphcuts(*image);

    // // Realizar segmentação usando Graph Cuts
    // vector<vector<int>> segments = segment_image_graphcuts(g, image->width, image->height);

    // // Salvar resultados
    // saveSegmentedImage("output_segmented_graphcuts", segments, *image);

    delete image;
    return 0;
}
