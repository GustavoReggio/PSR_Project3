# PSR_Project3
Terceiro trabalho de PSR
[![NPM](https://img.shields.io/badge/:badgeContent)](https://github.com/GustavoReggio/PSR_Project3/blob/main/LICENSE)
## Group members - G3

|  Num Mec | Nome                | email                   |
| ----: | ------------------- | ----------------------- |
| 118485|   Gustavo Reggio    |  gustavo.reggio@ua.pt   |  
| 105037|   Gonçalo Ornelas   |  goncalo.ornelas0@ua.pt |
| 118200|   Miguel Simões     |  miguelbsimoes@ua.pt    |


## Structure of navigation files:
**partI** - where you have all the files <br>
**images** - where the images are stored <br>
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

## Bibliografia
 - https://www.w3schools.com/git/default.asp
 - https://www.w3schools.com/xml/dom_nodes.asp
 - https://opencv.org/
