# Controle do robô por CLI

A aplicação foi desenvolvida para facilitar o acesso e a operação do robô, permitindo conexão via SSH, teleoperação, mapeamento de ambientes e navegação autônoma. Abaixo estão descritos os principais componentes e funcionalidades do projeto.

## 1. Estrutura do Projeto

A aplicação consiste em um único arquivo, devido à sua baixa complexidade.

A aplicação foi desenvolvida em Python utilizando as seguintes bibliotecas:

- **Click**: Para exibir uma interface de linha de comando amigável.
- **Inquirer**: Para criar um menu interativo.
- **Paramiko**: Para estabelecer conexões SSH.
- **Threading**: Para operações assíncronas, como o `bringup` do robô.
- **Subprocess**: Para executar comandos de teleoperação.
- **ROS 2 (rclpy)**: Para comunicação com o robô e controle de navegação.
- **Turtlesim e Nav2**: Para simulação e controle de navegação.

## 2. Funcionalidades e Módulos

### 2.1 Conexão com o Robô via SSH

- **Descrição**: A função `conectar_robo()` estabelece uma conexão SSH com o TurtleBot utilizando a biblioteca `Paramiko`.
- **Processo**: Após a conexão, um thread é iniciado para executar o comando `ros2 launch turtlebot3_bringup robot.launch.py`. Ele permite com que o robô comece a "escutar" tópicos, permitindo com que seja utilizado os serviços de teleoperação e movimentação de forma autônoma.

### 2.2 Teleoperação do Robô

- **Descrição**: A função `teleoperar_robo()` permite ao usuário controlar o robô usando manualmente o teclado.
- **Processo**: A teleoperação é implementada com o comando `ros2 run turtlebot3_teleop teleop_keyboard`, que aceita comandos de movimento em tempo real.

### 2.3 Navegação Autônoma

- **Descrição**: A função `modo_autonomo()` executa a navegação autônoma no ambiente, movendo o TurtleBot entre pontos de interesse definidos como `landmarks`.
- **Processo**:
  1. O usuário escolhe o ambiente (Ateliê, Laboratório, Sala de Aula).
  2. O sistema ROS é iniciado, e o TurtleBot navega entre os pontos estabelecidos (`landmarks`).
  3. Utiliza o `BasicNavigator` para se mover autonomamente e atualizar a posição do robô.
- **Landmarks**: São coordenadas definidas para cada ambiente, usadas para guiar o robô.

### 2.5 Encerramento da Comunicação

- **Descrição**: A função `encerrar_comunicacao()` fecha a conexão SSH com o robô.
- **Processo**: Fecha a sessão ativa do SSH e encerra qualquer atividade de comunicação com o TurtleBot.

## 3. Considerações Finais

O desenvolvimento dessa CLI permite um controle completo do TurtleBot para diferentes operações. A estrutura modular facilita a manutenção e o entendimento do código.
