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
**project_bringup** - Contém os ficheiros .launch que arrancam o gasebo e o reviz para ter spawn do robô. <br>
**project_descriprion** - Contém os ficheiros que dão forma ao robô e c ria os sensores e câmeras.<br>
**project_navigation** - Contém o mapa salvo e as diversas formas de navegação.
**imagens** - Pasta que contém as imagens. <br>

## Ordem de execução do Projeto:
**Parte 1** - Lançar os ficheiros "launch". Quarto e robô terão sues "spawns". <br>
**Parte 2** - Criar robô com respectios sensores e câmeras. <br>
**Parte 3** - Mapear Casa <br>
**scripts** - where the scripts are saved and where you can create virtual environments (env) as well

## Instruções para aceder o repositório:
```
 cd ~/catkin_ws/src/
 git clone https://github.com/GustavoReggio/PSR_Project3.git
 cd PSR_Project3
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

### Para controlar o robo pelo teclado:
```
rosrun project_navigation teleop
```

## Links de Apoio:
 - https://www.w3schools.com/git/default.asp
 - https://www.w3schools.com/xml/dom_nodes.asp
 - https://opencv.org/
