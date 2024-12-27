# Prova de conceito de robô de serviço

Este documento apresenta a prova de conceito (POC) do nosso robô de serviço autônomo capaz de navegar em ambientes internos utilizando integração com WhatsApp para controle e comunicação. O sistema implementa funcionalidades avançadas, como cartografia, navegação autônoma, e processamento de comandos recebidos via mensagens de texto ou áudio. 

## 1. Sistema de Cartografia

O sistema de cartografia é responsável por criar um mapa digital do ambiente onde o robô opera. Sensores como **LIDAR** e câmeras capturam dados ambientais, que são processados por algoritmos de cartografia. Esses mapas são fundamentais para navegação autônoma eficiente, permitindo ao robô planejar rotas e evitar obstáculos. A inicialização do sistema utiliza o comando:

```python
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/grupo2/modulo_8/2024-2B-T08-EC08-G01/src/cli/map/lab.yaml
```

Esse processo configura o robô para carregar o mapa do ambiente e ativar os módulos de navegação.

## 2. Sistema de Localização e Mapeamento Simultâneo (SLAM)

O **SLAM** (Simultaneous Localization and Mapping) permite que o robô construa e atualize um mapa do ambiente enquanto rastreia sua posição. Dados de sensores são combinados com algoritmos de processamento para identificar características do ambiente e ajustar a trajetória em tempo real. A inicialização do SLAM é feita com o seguinte trecho de código:

```python
initial_pose = PoseStamped()
initial_pose.header.frame_id = "map"
initial_pose.pose.position.x = 0.4
initial_pose.pose.position.y = 0.03
nav.setInitialPose(initial_pose)
nav.waitUntilNav2Active()
```

Esse processo posiciona o robô no ambiente e ativa o módulo de navegação para respostas dinâmicas a mudanças.

## 3. Controle via WhatsApp

O sistema é integrado ao **WhatsApp** para controle remoto. Mensagens de texto ou áudio enviadas ao robô são processadas, permitindo que o robô execute tarefas como navegação até locais específicos. O processamento das mensagens é gerenciado com **Twilio** e integração com APIs, como demonstrado no trecho:

```python
@app.post('/whats_msg')
async def bot(Body: str = Form(...), MediaUrl0: str = Form(None)):
    incoming_msg = Body.lower()
    resp = MessagingResponse()
    msg = resp.message()
    if MediaUrl0:
        media_content = requests.get(MediaUrl0).content
        strMsg = speech_to_text("audio.wav")
    else:
        strMsg = incoming_msg
    response = client.chat.completions.create(model="gpt-4", messages=[{"role": "user", "content": strMsg}])
```

Essa funcionalidade permite comandos intuitivos, como "ir para a sala de reunião," que são interpretados e transformados em ações de navegação.

## 4. Navegação Autônoma e Execução de Tarefas

O robô realiza navegação autônoma para destinos específicos utilizando um modelo de coordenadas e poses definidas. Um exemplo de navegação é mostrado abaixo:

```python
def modo_autonomo(x, y, z):
    goal_pose = create_pose_stamped(nav, x, y, z)
    nav.followWaypoints([goal_pose])
    while not nav.isTaskComplete():
        print(nav.getFeedback())
```

Esse módulo utiliza algoritmos de controle e otimização para garantir que o robô alcance o destino de forma segura e eficiente.

## 5. Registro de Tarefas e Histórico

Todas as solicitações recebidas e concluídas são registradas em um banco de dados sincronizado com **Turso**. Este registro é crucial para auditoria e monitoramento. O exemplo de registro é:

```python
r = requests.post("http://0.0.0.0:8000/log/criar?origem_dado=1&destino_dado=2&id_usuario=4&concluido=0")
```

---

Essa prova de conceito demonstra a viabilidade de um robô de serviço controlado remotamente, integrando tecnologias de ponta como ROS, FastAPI, e GPT para oferecer uma solução robusta e escalável.
