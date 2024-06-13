# Detecção de objetos e marcação em mapa - inspection_yolov5

Repositório utilizado para desenvolver o código-fonte da detecção de objetos no ROS2 e marcações em um mapa deles. Algoritmo desenvolvido para a execução do projeto FAPESP 2023/06578-6, sob orientação do Prof. Dr. Roberto Inoue.

Atualmente, o algoritmo desenvolvido realiza a detecção de extintores de incêndio.

## Reconhecimento
Para a aplicação da rede treinada no ROS2, o código desenvolvido por [RobotMania](https://www.youtube.com/watch?v=ALD9KfCfZk4) foi adaptado para as condições atuais, retirando a orientação de bouding boxes e os arquivos de treinamento. Algumas outras alterações em códigos internos também foram necessárias.

## Requisitos
Este projeto foi desenvolvido e aplicado utilizando os seguintes itens: 
- Jetson Xavier NX,
- Ubuntu 20.04 do SDK da NVIDIA,
- ROS2 Foxy,
- Zedm camera,
- YOLOv5,
- Robô móvel terrestre presente no LARIS.

Para a instalação dos requisitos, siga os passos a seguir:
- Instalação do ROS2 Foxy a partir deste [link](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html);
- Instalação do pacote ZED Wrapper para ROS2 neste [link](https://github.com/stereolabs/zed-ros2-wrapper) em um workspace de sua escolha;
- Instalação dos pré-requisitos para a [YOLOv5](https://github.com/ultralytics/yolov5). Vale ressaltar que para placas NVIDIA, é necessário instalar as versões específicas para o torch e torchvision. Para mais detalhes ver a seção de instalação na Wiki

## Instalação

A instalação segue a mesma estrutura de um pacote convencional ROS2 pelos seguintes passos:
```bash
mkdir -p ~/your_ros2_wksp/src/ # create your workspace if it does not exist
cd ~/your_ros2_wksp/src/ #use your current ros2 workspace folder
git clone https://github.com/joaocarloscampi/inspection_yolov5.git

cd ..
sudo apt update
colcon build --symlink-install

source ~/your_ros2_wksp/install/setup.bash
```

## Estrutura do repositório

| Diretório | Descrição |
|-----------|-----------|
| launch          | Contém launch files para execução do pacote ROS2. |
| scripts         | Contém o código fonte da detecção de objetos. |
| scripts/docs    | Documentações remanescentes da aplicação original (Não utilizado) |
| scripts/models  | Pasta nativa da YOLOv5. |
| scripts/tools   | Pasta com arquivos da aplicação original (TODO: Verificar necessidade).|
| scripts/utils   | Parta nativa da YOLOv5, com mais alguns arquivos para auxiliar a execução da detecção. |

## Execução dos algoritmos
aaaa
Lembrar dos chmod +x
