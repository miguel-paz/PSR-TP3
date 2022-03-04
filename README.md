# Programação em Sistemas Robóticos - Trabalho 3 - Team Hunt e Corrida Manual na FNR

## Sumário


  Este projeto consiste no clássico jogo de Team Hunt e numa corrida contra o relógio na FNR.
    Na primeira parte do trabalho, a equipa vermelha caça a verde, a equipa azul a vermelha e por fim a equipa verde terá de caçar a azul. Consequentemente, 
    a equipa vermelha terá de fugir da equipa azul, a equipa azul da equipa verde e a equipa verde da vermelha.
    A segunda parte parte do trabalho consiste em fazer percorrer, manualmente, um robõ, pelo circuito da feira nacional de robótica fornecido pelo 
    João Bernardo, investigador do IEETA.
    
## Explicação da configuração do jogo

![Image](https://cdn.discordapp.com/attachments/943226390097035335/949189994570997790/PSR_INITIAL_DRAFT.drawio.png)

A figura anterior, explica o processo de mudança de estados dos robôs e como se processa a mudança. 
O estado de wandering é o estado em que o robô simplesmente vagueia pelo mapa, não colidindo com objetos e obstáculos que possam aparecer. Se um robô avistar a sua 
presa este passa para o estado de hunting e não deixa este estado até o caçar, deixar de ver a sua presa ou avistar o seu predador. 

Caso esta última aconteça, este muda automaticamente do estado de hunting para o estado de fleeing. Para sair deste estado, não pode estar à vista do seu predador.

Quanto aos estados de teleop e de goal, o primeiro refere-se ao controlo manual do robô e o segundo refere-se a um estado em que o utilizador determina uma posição 
à qual o robô tem de se dirigir.

## Configuração dos robôs

O modelo utilizado neste trabalho foi o turtlebot3 waffle pi. Optámos por meter ,além de uma camâra frontal, uma camâra dianteira para ser mais realista.


## Códigos para iniciar o jogo da apanhada

#Arenas:
roslaunch p_mcarvalho_bringup tp3_gazebo_arena_1.launch
roslaunch p_mcarvalho_bringup tp3_gazebo_arena_2.launch
roslaunch p_mcarvalho_bringup tp3_gazebo_arena_3.launch

#Pista FNR:
roslaunch p_mcarvalho_bringup fnr_map.launch

#Apenas um robô:
roslaunch p_mcarvalho_bringup tp3_bringup.launch

#3 Robôs e Árbitro em terminal à parte:
roslaunch p_mcarvalho_bringup tp3_bringup_multiple.launch

#Apenas Árbitro:
rosrun th_referee th_referee.py

#Lançamento de inteligência artifical de robôs: 
rosrun p_mcarvalho_player primal_agent __name:="nome do robô"
 p.e:
 rosrun p_mcarvalho_player primal_agent __name:=red1

#Condução manual através de telemóvel: 
roslaunch p_mcarvalho_bringup teleop.launch

#Lançamento do nó coach:
rosrun p_mcarvalho_bringup coach.py __name:=red_coach
