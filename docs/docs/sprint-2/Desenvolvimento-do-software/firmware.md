# Desenvolvimento do Software

## 1. Configuração das Dependências

Para a realização da movimentação autônoma do robô, foi necessário realizar a configuração inicial do robô, assim como preparar o ambiente para que ele pudesse ser capaz de funcionar da forma desejada. Nesse sentido, a primeira etapa foi a instalação das dependências na máquina dos desenvolvedores e no robô. Abaixo segue o passo a passo que deve ser executado para poder recriar o projeto:

:::warning
Esse tutorial funcionara apenas no Ubuntu e apenas em versões que o Ros-Humble esteja disponível.
:::

1 - Abra o seu terminal e execute os próximos passos.

2 - `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*`

3 - `sudo apt install ros-humble-rmw-cyclonedds-cpp`

4 - `echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc`

### 1.1 Ajustando pacotes

A seguir, vamos mexer na configuração padrão do turtlebot para que ele use o método adequado de localização.

5 - Navegue até o diretório `/opt/ros/humble/share/turtlebot3_navigation2/param`

6 - Agora é necessário alterar o arquivo `burger.yaml`, para isso digite `nano burger.yaml` ou utilize o editor de texto de sua preferência.

7 - Encontre a linha que diz: `robot_model_type: "differential"` e modifique para `robot_model_type: "nav2_amcl::DifferentialMotionModel"`.

Após a execução desses passos, é possível utilizar dos pacotes do turtlebot disponíveis no ROS. Para um maior entendimento do que é ros, clique [aqui](https://docs.ros.org/)

:::tip
Outro passo importante para a execução do projeto, é que tanto o computador utilizado para desenvolvimento quanto o robô tenham o mesmo `ROS_DOMAIN_ID`. Para verificar isso, execute `echo $ROS_DOMAIN_ID` em ambos os ambientes. Caso não esteja definido ou os números divirjam, adicione no `.bashrc` a seguinte linha: `export ROS_DOMAIN_ID=<Número Desejado>`
:::

## 2. Execução do Projeto

### 2.1 Criando um mapa

Para criar um robô autônomo, é necessário mapear o ambiente antes. Sendo assim, o tutorial a seguir mostra os comandos utilizados pelo grupo para realizar essa tarefa:

1 - Execute no terminal `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

Esse comando irá abrir uma janela com o simulador gazebo, o qual emula um TurtleBot3. Caso você esteja com o robô ligado, também funcionará diretamente nele.

2 - Execute `ros2 run turtlebot3_teleop teleop_keyboard` para começar a teleoperação do robô.

3 - `ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False`. Entretanto, caso deseje fazer com o robô emulado troque a parte `use_sim_time:=False` para `use_sim_time:=True`

:::info
Em alguns momentos o grupo encontrou dificuldade quando utilizando o parâmetro `use_sim_time`, dessa forma, caso não funcione, utilize o comando sem o parâmetro.
:::

Após executar esse último comando, deverá abrir uma janela do Rviz, a qual indica o início do mapeamento. Dessa forma, é necessário movimentar o robô pelo ambiente para que ele possa desenhar um mapa capaz de representar fielmente os obstáculos.

Após de se movimentar por todo o ambiente desejado é necessário executar o comando: `ros2 run nav2_map_server map_saver_cli -f <nome-do-mapa>` para salvar o mapa e usá-lo depois.

:::tip
nome-do-mapa pode ser substituído por um nome ou um caminho de uma pasta de sua preferência.
:::

A seguir, um exemplo de como poderia ficar o comando:

```
mkdir -p ~/Documents/Maps
ros2 run nav2_map_server map_saver_cli -f ~/Documents/Maps/my-map
```

### 2.2 Navegando pelo mapa pelo RVIZ

1 - Agora vamos navegar pelo mapa. No caso, vamos executar `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

2 - Em outro terminal execute `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<arquivo-do-mapa>.yaml`

Com esses dois comandos, é para aparecer as aplicações Gazebo e Rviz na tela. Nesse sentido, fique na tela do Rviz, pois por lá executaremos para o robô se movimentar.

3 - Agora na tela do Rviz no `2D Pose Estimate` e selecione uma direção e depois clique no `Nav2 Goal` e selecione uma direção de destino. Com isso você verá o robô se movendo de forma autônoma até o ponto desejado.

Esses passos foram executados durante essa Sprint para o robô conseguir se locomover de maneira autônoma. Assim, todas as instruções foram baseadas no material de Rodrigo Nicola, localizado em: [https://rmnicola.github.io/m8-ec-encontros/turtleslam/](https://rmnicola.github.io/m8-ec-encontros/turtleslam/)

## 3. Referências:

[1] SLAM com Turtlebot | M8 (EC). Github.io. Disponível em: [https://rmnicola.github.io/m8-ec-encontros/turtleslam/](https://rmnicola.github.io/m8-ec-encontros/turtleslam/). Acesso em: 10 nov. 2024.
‌
