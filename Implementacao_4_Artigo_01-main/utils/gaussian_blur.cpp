
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
void saveAsPPM(const cv::Mat& image, const std::string& filename) {
    std::ofstream ppmFile(filename);

    if (!ppmFile) {
        std::cerr << "Erro ao criar o arquivo PPM!" << std::endl;
        return;
    }

    // Escrever o cabeçalho
    ppmFile << "P3\n";
    ppmFile << image.cols << " " << image.rows << "\n";
    ppmFile << "255\n";  // Valor máximo de cor

    // Escrever os dados de pixel
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            cv::Vec3b color = image.at<cv::Vec3b>(y, x);
            ppmFile << (int)color[2] << " "  // R
                    << (int)color[1] << " "  // G
                    << (int)color[0];        // B
            if (x != image.cols - 1) {
                ppmFile << " ";
            }
        }
        ppmFile << "\n";
    }

    ppmFile.close();
    std::cout << "Imagem salva como " << filename << std::endl;
}


void applyGaussianBlur(const std::string& inputImage, const std::string& outputImage, int kernelSize, double sigma) {
    // Carregar a imagem de entrada
    cv::Mat image = cv::imread(inputImage, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Erro ao carregar a imagem!" << std::endl;
        return;
    }

    // Aplicar o filtro Gaussiano
    cv::Mat blurredImage;
    cv::GaussianBlur(image, blurredImage, cv::Size(kernelSize, kernelSize), sigma);

    // Salvar a imagem resultante em formato PPM
    saveAsPPM(blurredImage, outputImage);

    // Exibir a imagem original e a imagem borrada
    cv::imshow("Imagem Original", image);
    cv::imshow("Imagem Borrada", blurredImage);

    // Esperar uma tecla para fechar as janelas
    cv::waitKey(0);
    cv::destroyAllWindows();
}

// Função para salvar uma imagem no formato PPM
int main(int argc, char** argv) {
    // Verificar se o número de argumentos é suficiente
    if (argc != 5) {
        std::cerr << "Uso: " << argv[0] << " <inputImage> <outputImage> <kernelSize> <sigma>" << std::endl;
        return -1;
    }

    // Obter os parâmetros da linha de comando
    std::string inputImage = argv[1];
    std::string outputImage = argv[2];
    int kernelSize = std::stoi(argv[3]);
    double sigma = std::stod(argv[4]);

    // Validar o tamanho do kernel
    if (kernelSize % 2 == 0) {
        std::cerr << "O tamanho do kernel deve ser ímpar!" << std::endl;
        return -1;
    }

    // Aplicar o filtro Gaussiano
    applyGaussianBlur(inputImage, outputImage, kernelSize, sigma);

    return 0;
}
