#include <numeric>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

Mat displayImage;  // A imagem que será exibida
vector<Point> objectSeeds, backgroundSeeds;  // Listas de seeds do objeto e fundo
string filename = "../images/torre-eiffel.ppm";

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

// Função para capturar as seeds do objeto e do fundo, recebendo o caminho do arquivo
void capturarSeeds(const string& filename, vector<Point>& objS, vector<Point>& bkgS) {
    // Carregar a imagem dentro da função
    Mat image = imread(filename);
    if (image.empty()) {
        cerr << "Erro ao carregar a imagem.\n";
        return;
    }

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
        objS = objectSeeds;
        bkgS = backgroundSeeds;
        if (waitKey(30) == 27) break; // Pressione 'ESC' para sair
    }
}

void build_histograms(std::vector<double> &histogram_obj, 
                      std::vector<double> &histogram_bkg, vector<int> &labels, int size) {
    histogram_obj.resize(256, 0);
    histogram_bkg.resize(256, 0);
    labels.resize(size, -1);

    std::vector<Point> objS, bkgS;
    capturarSeeds(filename, objS, bkgS);

    Mat image = imread(filename, IMREAD_COLOR);
    if (image.empty()) {
        cerr << "Erro ao carregar a imagem.\n";
        return;
    }

    int objCount = 0, bkgCount = 0;

    for (const auto &p : objS) {
        if (p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows) {
            Vec3b color = image.at<Vec3b>(p.y, p.x);
            int intensity = round(0.299 * color[2] + 0.587 * color[1] + 0.114 * color[0]);
            histogram_obj[intensity]++;
            labels[p.y * image.cols + p.x] = 1;
            objCount++;
        }
    }

    for (const auto &p : bkgS) {
        if (p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows) {
            Vec3b color = image.at<Vec3b>(p.y, p.x);
            int intensity = round(0.299 * color[2] + 0.587 * color[1] + 0.114 * color[0]);
            histogram_bkg[intensity]++;
            labels[p.y * image.cols + p.x] = 0;
            bkgCount++;
        }
    }

    if (objCount > 0) {
        for (auto &value : histogram_obj) {
            value /= objCount;
        }
    }

    if (bkgCount > 0) {
        for (auto &value : histogram_bkg) {
            value /= bkgCount;
        }
    }

    // Optional: Print labels for debugging
    // for (int y = 0; y < image.rows; y++) {
    //     for (int x = 0; x < image.cols; x++) {
    //         int index = y * image.cols + x;
    //         cout << labels[index] << " ";
    //     }
    //     cout << endl;
    // }
}
