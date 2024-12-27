import re
from fastapi import FastAPI, Form, HTTPException
from fastapi.responses import Response
from twilio.twiml.messaging_response import MessagingResponse
import requests
import rclpy
from geometry_msgs.msg import Twist
from openai import OpenAI
import json
import os
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
from pydantic import BaseModel
from geometry_msgs.msg import PoseStamped
import paramiko
import threading
from dotenv import load_dotenv
import uvicorn
import logging
from STT import speech_to_text
import soundfile as sf
import libsql_experimental as libsql
import time

app = FastAPI()
load_dotenv()

seq = {}

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
ssh = None

logger = logging.getLogger("my-fastapi-app")

conn = None

def connect():
    url = os.getenv("TURSO_DB_URL")
    auth_token = os.getenv("TURSO_DB_TOKEN")
    conn = libsql.connect("ceciadb.db", sync_url=url, auth_token=auth_token)
    conn.sync()
    return conn

def add_sequencia(fila):
    for key in fila.keys():
        if key not in seq.keys():
            seq[key] = [fila[key][0],fila[key][1]]

@app.on_event("startup")
def startup_event():
    rclpy.init()
    global node, velocity_publisher, nav, conn
    
    initializeSSH()
    nav = BasicNavigator()
    logger.info("BRINGUP EXECUTADO")
    initiateNav()
    node = rclpy.create_node('turtlebot_whatsapp_controller')
    velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    
    # url = os.getenv("TURSO_DB_URL")
    # auth_token = os.getenv("TURSO_DB_TOKEN")
    # conn = libsql.connect("ceciadb.db", sync_url=url, auth_token=auth_token)
    # conn.sync()

@app.on_event("shutdown")
def shutdown_event():
    rclpy.shutdown()

def run_bringup():
    global ssh
    try:
        bringup_command = (
            "source /opt/ros/humble/setup.bash && "
            "export LDS_MODEL=LDS-02 && "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            "export ROS_DOMAIN_ID=109 && "
            "export TURTLEBOT3_MODEL=burger && "
            "source ~/turtlebot3_ws/install/setup.bash && "
            "source /opt/ros/humble/setup.bash && "
            "ros2 launch turtlebot3_bringup robot.launch.py && "
            "ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/grupo2/modulo_8/2024-2B-T08-EC08-G01/src/cli/map/lab.yaml"
        )
        stdin, stdout, stderr = ssh.exec_command(bringup_command, get_pty=True)
        print("Executando bringup do TurtleBot...")
        logger.info("Executando bringup do Turtlebot...")
    except Exception as e:
        print(f"Erro ao executar o bringup: {e}")

def initializeSSH():
    global ssh, bringup_thread

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(hostname="grupo2.local", username="grupo2", password="Repipe2")
        bringup_thread = threading.Thread(target=run_bringup)
        bringup_thread.daemon = True
        bringup_thread.start()

    except Exception as e:
        print(f"Erro ao conectar ao TurtleBot via SSH: {e}")


def send_turtlebot_command(linear=0.0, angular=0.0):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    velocity_publisher.publish(vel_msg)

def initiateNav():
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.4
    initial_pose.pose.position.y = 0.03
    initial_pose.pose.position.z = 0.00147
    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

def create_pose_stamped(navigator, pos_x, pos_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.position.z = rot_z
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def modo_autonomo(x, y, z):
    goal_pose = create_pose_stamped(nav, x, y, z)
    nav.followWaypoints([goal_pose])

    while not nav.isTaskComplete():
        print(nav.getFeedback())

class NavigationCommand(BaseModel):
    place_name: str

@app.post('/whats_msg')
async def bot(Body: str = Form(...), MediaUrl0: str = Form(None)):
    global conn
    incoming_msg = Body.lower()
    resp = MessagingResponse()
    msg = resp.message()
    strMsg = ""
    try:
        if MediaUrl0:
            media_content = requests.get(
                MediaUrl0,
                auth=(os.getenv("TWILIO_ACCOUNT_SID"), os.getenv("TWILIO_AUTH_TOKEN"))
            ).content
            with open("audio_message.ogg", "wb") as f:
                f.write(media_content)
            data, samplerate = sf.read("audio_message.ogg")
            sf.write("audio.wav", data, samplerate)
            strMsg = speech_to_text("audio.wav")
        else:
            strMsg = incoming_msg
    except Exception as e:
        print(f"Error processing media: {e}")
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {
                "role": "system",
                "content": (
                    "Você é um assistente de controle de robô. O usuário pode pedir para ir a um local. "
                    "Quando o usuário pede para ir a um local, chame a função 'navigate_robot' com o nome do local. "
                    "Não forneça diretamente as coordenadas. Apenas forneça o nome do local."
                ),
            },
            {"role": "user", "content": strMsg},
        ],
        functions=[
            {
                "name": "navigate_robot",
                "description": "Mova o robô para uma localização específica informando o nome do local.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "place_name": {"type": "string", "description": "O nome do local de destino."}
                    },
                    "required": ["place_name"]
                },
            }
        ],
    )
    conn = connect()
    if response.choices and response.choices[0].message and response.choices[0].message.function_call:
        function_call = response.choices[0].message.function_call
        if function_call.name == "navigate_robot":
            args = json.loads(function_call.arguments)
            place_name = args["place_name"].lower()
            result = conn.execute(f"SELECT id FROM posicoes WHERE lugar = '{place_name}'").fetchone()
            print(f"Resultado: {result}")
            if result:
                try:
                    r = requests.post(
                        f"http://0.0.0.0:8000/log/criar?origem_dado=1&destino_dado={result[0]}&id_usuario=4&concluido=0"
                    )
                    if r.status_code == 200:
                        msg.body("Pedido adicionado na fila.")
                    else:
                        msg.body(f"Erro ao adicionar na fila. Status: {r.status_code}")
                except Exception as e:
                        msg.body(f"Erro ao processar requisição POST: {e}")
    else:
        msg.body(response.choices[0].message.content)
    # r = requests.get(f"http://0.0.0.0:8000/log/adicionar_fila")
    # mover()
    return Response(content=str(resp), media_type="application/xml")


@app.get("/navigate")
def mover():
    try:
        conn = connect()
        while seq != {}:
            id_log = list(seq.keys())[0]
            ambiente = seq.pop(id_log)
            for i in range(2):
                result = conn.execute(f"SELECT posicao_x, posicao_y, posicao_z FROM posicoes WHERE id = {int(ambiente[i])}").fetchall()
                print(f"Resultado: {result}")
                if result:
                    x, y, z = result[0]
                    modo_autonomo(x, y, z)
                    time.sleep(2)
            r = requests.put(f"http://0.0.0.0:8000/log/atualizar/{int(id_log)}?concluido=1")
        return {"message": "Navegação concluida"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao mover o robô: {str(e)}")

class Sequence(BaseModel):
    info:dict

@app.post("/add_seq")
async def add_seq(json: Sequence):
    try:
        add_sequencia(json.info)
        return {"message": f"Logs adicionados na fila com sucesso!"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao adicionar os logs na fila: {str(e)}")
    
@app.get("/list_seq")
def sla():
    return seq


if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5000)