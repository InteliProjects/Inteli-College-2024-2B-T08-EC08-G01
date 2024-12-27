# Avanços no desenvolvimento do robô 

Durante essa sprint foi implementadas melhoras no desenvolvimento do robô, focando em implementar de fato a movimentação completamente autônoma, utilizando o `BasicNavigator` como forma de realizar isso. Ademais, foi realizada a integração com o sistema de mensagem por `Whatsapp` utilizando o `Twilio`. 

## Desenvolvimento da CLI 

### Inicialização do Robô

Para finalizar a cli, foi implementado uma função de inicialização para declarar a posição do robô no mapa uma única vez, assim resolvendo o problema de que em toda navegação era necessário guardar a posição anterior.

Trecho do código com ela implementada: 

```python
def initiateNav(nav):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = landmarks[0][0]
    initial_pose.pose.position.y = landmarks[0][1]
    initial_pose.pose.position.z = landmarks[0][2]
    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()
    return nav
```

### Waypoints

Para facilitar a movimentação foi implementado a função de waypoints, [disponibilizada pelo professor Rodrigo Nicola](https://rmnicola.github.io/m8-ec-encontros/turtleslam#44-navegando-atrav%C3%A9s-de-waypoints). Isso se deve pois a função permite com que o robô possa se movimentar a mais de um ponto em uma única chamada de função. 

### Arquivo Utils.py

Para uma melhor organização do grupo, foi criado o arquivo `utils.py`, o qual guarda a função de criar uma pose, caso necessário, e os pontos que foram decidos para representar o ambiente do hospital. Idealmente, isso se tornará uma parte do banco de dados na sprint seguinte. 

## Implementação do Twilio 

As funções citadas anteriormente, foram implementadas no sistema do twilio, assim como na cli. Sendo assim, a movimentação do robô ocorre por waypoints via whatsapp. Para isso, o sistema recebe um input de texto, contendo os pontos que ele deseja para onde o robô se locomova. Após isso, esse comando é enviado para a api do `ChatGpt`, a qual decide o comando que deve ser utilizado, Navegação ou Inicialização (O último esta para ser implementado até a conclusão dessa sprint.), e retorna o resultado. Dessa forma, é ativada a função de inicialização do mapa do robô e ele se locomove até o local desejado.