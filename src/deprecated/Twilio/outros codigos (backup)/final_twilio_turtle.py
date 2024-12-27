from flask import Flask, request
from twilio.twiml.messaging_response import MessagingResponse
import requests
import rclpy
from geometry_msgs.msg import Twist

app = Flask(__name__)

# Inicializa o cliente ROS 2
rclpy.init()

# Cria um nó ROS 2 para publicar comandos de movimento para o TurtleBot3
node = rclpy.create_node('turtlebot_whatsapp_controller')
velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

# Função para enviar comandos de movimento para o TurtleBot3
def send_turtlebot_command(linear=0.0, angular=0.0):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    velocity_publisher.publish(vel_msg)

# Rota para receber mensagens do Twilio
@app.route('/bot', methods=['POST'])
def bot():
    incoming_msg = request.values.get('Body', '').lower()
    resp = MessagingResponse()
    msg = resp.message()
    responded = False

    # Salva o arquivo de áudio da mensagem de voz
    media_url = request.values.get('MediaUrl0', '')
    if media_url:
        media_content = requests.get(
            media_url,
            auth=('TWILIO_ACCOUNT_SID', 'TWILIO_AUTH_TOKEN')
        ).content
        with open("audio_message.ogg", "wb") as f:
            f.write(media_content)

    # Comandos padrão do Twilio
    if 'citação' in incoming_msg:
        r = requests.get('https://api.quotable.io/random')
        if r.status_code == 200:
            data = r.json()
            quote = f'{data["content"]} ({data["author"]})'
        else:
            quote = 'Não consegui recuperar uma citação neste momento, desculpe.'
        msg.body(quote)
        responded = True
    elif 'gato' in incoming_msg or 'gata' in incoming_msg:
        msg.media('https://cataas.com/cat')
        responded = True
    # Comandos para o TurtleBot3
    elif 'andar' in incoming_msg:
        send_turtlebot_command(linear=0.2)  # Mover para frente a 0.2 m/s
        msg.body("TurtleBot3 está andando para frente!")
        responded = True
    elif 'parar' in incoming_msg:
        send_turtlebot_command()  # Parar movimento
        msg.body("TurtleBot3 parou!")
        responded = True
    elif 'girar' in incoming_msg:
        send_turtlebot_command(angular=0.5)  # Rotacionar a 0.5 rad/s
        msg.body("TurtleBot3 está girando!")
        responded = True

    if not responded:
        msg.body('Só conheço frases, gatos e alguns comandos para o TurtleBot3!')

    return str(resp)

if __name__ == '__main__':
    try:
        app.run()
    finally:
        rclpy.shutdown()
