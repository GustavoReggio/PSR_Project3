# PSR_Project3
Terceiro trabalho de PSR
[![NPM](https://img.shields.io/badge/:badgeContent)](https://github.com/GustavoReggio/PSR_Project3/blob/main/LICENSE)
## Nome dos participantes do Projeto - G3

|  Num Mec | Nome                | email                   |
| ----: | ------------------- | ----------------------- |
| 118485|   Gustavo Reggio    |  gustavo.reggio@ua.pt   |  
| 105037|   Gonçalo Ornelas   |  goncalo.ornelas0@ua.pt |
| 118200|   Miguel Simões     |  miguelbsimoes@ua.pt    |

## Descrição dos Pacotes:
**project_bringup**     - Contém os ficheiros .launch que arrancam o gasebo e o reviz para ter spawn do robô. <br>
**project_descriprion** - Contém os ficheiros que dão forma ao robô e c ria os sensores e câmeras.<br>
**project_navigation**  - Contém o mapa salvo e as diversas formas de navegação.<br>
**imagens**             - Pasta que contém as imagens. 

## Ordem de execução do Projeto:
**Parte 1** - Lançar os ficheiros "launch". Quarto e robô terão sues "spawns". <br>
**Parte 2** - Criar robô com respectios sensores e câmeras. <br>
**Parte 3** - Mapear Casa. <br>
**Parte 4** - Spawn de Objetos. <br>
**Parte 5** - Missões Passivas: <br>
- Mover para quarto, mover para sala;
- Procurar uma esfera vermelha no quarto pequeno;
- Verificar se o computador portátil está no escritório;
- Verificar se está alguém no quarto grande;
- Verificar se a mesa da sala está levantada (livre de objetos);
- Fotografar a divisão X;
- Verificar se alguém está em casa;
- Contar o número de cubos azuis existentes em casa.<br>

**Parte 6** - Missões Ativas (Avançadas).<br>
- Tocar no objeto X;
- Deitar o objeto X abaixo da mesa na divisão Y;
- Agarrar a lata de coca-cola do quarto e colocá-la no balde do lixo do escritório.<br>

## Instruções para aceder o repositório:
```
 cd ~/catkin_ws/src/
 git clone https://github.com/GustavoReggio/PSR_Project3.git
 cd PSR_Project3
 cc
 ```

## Para adicionar no ficheiro .barshrc
Para importar e spawn de objetos
```
export GAZEBO_MODEL_PATH=`rospack find project_description`/models/:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=`rospack find robutler_description_23-24`/models/:${GAZEBO_MODEL_PATH}
```
Em outro terminal Compilar ao final da adição dos exports:
```
cc
```

## Instruções para executar o projeto:

### Lançar o Gazebo:

```
roslaunch project_bringup gazebo.launch
```

### Lançar o Rviz:
```
roslaunch project_bringup bringup.launch    
```

### Para controlar o robô:
#### Por teclado:
```
rosrun project_navigation teleop
```
#### Por PS4 control:
Instalar:
```
git clone https://github.com/naoki-mizuno/ds4drv --branch devel
cd ds4drv
python2 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
cd ~/catkin_ws/src
git clone https://github.com/naoki-mizuno/ds4_driver.git
```
Compilar por:
```
roslaunch ds4_driver ds4_driver.launch
ou
rosrun ds4_driver ds4_driver_node.py
```

Lançar:
```
roslaunch ds4_driver demo.launch
```

## Links de Apoio:
 - https://www.w3schools.com/git/default.asp
 - https://www.w3schools.com/xml/dom_nodes.asp
 - https://opencv.org/
 - https://classic.gazebosim.org/tutorials?tut=ros_roslaunch
 - https://wiki.ros.org/ds4_driver#:~:text=Pair%20and%20connect%20to%20your,to%20connect%20to%20the%20device.
 - https://github.com/engcang/PS4_Joystick_teleop_Mobile_Robots_ROS_Python