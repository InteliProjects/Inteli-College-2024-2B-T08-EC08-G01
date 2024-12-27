# Implementação da movimentação autônoma

Durante essa sprint, foi realizado ajustes na implementação da cli, para que ela seja capaz de movimentar para pontos pré-determinados de um mapa, sem a necessidade de guiar o robô ao devido local. Caso deseje ver como foi implementada a cli, acesse a documentação da [Sprint 2](sprint-2/Desenvolvimento-do-software/cli.md)

### 1. Navegação Autônoma

- **Descrição**: A função `modo_autonomo()` executa a navegação autônoma no ambiente, movendo o TurtleBot entre pontos de interesse definidos como `landmarks`.
- **Processo**:
  1. O usuário escolhe o ambiente (Ateliê, Laboratório, Sala de Aula).
  2. O sistema ROS é iniciado, e o TurtleBot navega entre os pontos estabelecidos (`landmarks`).
  3. Utiliza o `BasicNavigator` para se mover autonomamente e atualizar a posição do robô.
- **Landmarks**: São coordenadas definidas para cada ambiente, usadas para guiar o robô.
