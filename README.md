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

## Arquivos de execução launch

| Comando | Descrição |
|---------|-----------|
| `ros2 launch inspection_yolov5 inspection_general.launch.py` | Inicia a detecção de objetos apresentando em uma janela pop-up |
| `ros2 launch inspection_yolov5 detection.launch.py` | Inicia a detecção de objetos apresentando em uma janela pop-up |
| `ros2 launch inspection_yolov5 launch_depth_detection.launch.py` | Inicia a estimativa de posição dos objetos no mapa com base na câmera. |

## Execução dos algoritmos
O primeiro launch é o responsável por executar a aplicação completa de detecção de objetos, aquisição da profundidade, estimativa de coordenadas em relação à camera e em relação ao mapa (por enquanto). Pode-se customizar o que deseja-se rodar a partir do arquivo `config/inspection_parameters.yaml`, como habilitar funções específicas ou mudar o nome dos tópicos a ser publicado.

O segundo launch apresentado é o responsável por fazer a detecção dos objetos com a rede treinada (utilizando o arquivo extintores.pt) e publicar as coordenadas da bouding boxes no tópico `yolov5_ros2/bounding_boxes`. O código `scripts/ros_detect.py` possui um parâmetro (manual por enquanto) definindo o tópico `/zedm/zed_node/left/image_rect_color` como fonte das imagens no ROS2. O script demora um pouco para iniciar, basta aguardar a janela aparecer que a detecção se inicia

Já o terceiro launch, a partir do código `scripts/depth_bouding_box.py` recebe as detecções do tópico `yolov5_ros2/bounding_boxes` e junta com as informações de depth do tópico `/zedm/zed_node/depth/depth_registered` e calcula a distância relativa à câmera e, na sequência, as coordenadas no mapa (TODO: Separar os launchs em detecção simples e detecção no mapa).

Para a detecção no mapa, algumas coisas são necessárias:
- Toda a árvore de transformações TF do robô, da câmera e do mapa deve ser criada antes da execução deste launch;
- É necessário definir quais são os frames de transformação. O frame de destino é o `map` e o de origem é `zedm_left_camera_optical_frame`;

Caso na hora de rodar os launchs o ROS2 indicar que não consegue encontrar os executáveis dos arquivos python, execute os comandos:
```bash
cd ~/your_ros2_wksp/src/inspection_yolov5/scripts

sudo chmod +x ros_detect.py
sudo chmod +x depth_bouding_box.py

cd ~/your_ros2_wksp
colcon build --symlink-install
```
