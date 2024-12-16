
# Filtro Gaussiano com OpenCV

Código para aplicar filtro gaussiano em uma aimagem

## Funcionalidade

- O programa carrega uma imagem em formato comum (como PPM ou PGM).
- Aplica um filtro gaussiano para suavizar a imagem.
- Salva a imagem borrada no formato PPM.
- Exibe tanto a imagem original quanto a imagem borrada.

## Dependências

- [OpenCV](https://opencv.org/releases/): Para manipulação de imagens e aplicação do filtro gaussiano.

#### Ubuntu/Debian

Para compilar com OpenCV, use o seguinte comando:

```bash
g++ -o gaussian_blur gaussian_blur.cpp `pkg-config --cflags --libs opencv4`
