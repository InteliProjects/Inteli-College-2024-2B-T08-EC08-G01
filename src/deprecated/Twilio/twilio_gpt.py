from fastapi import FastAPI, Form
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
from math import pi
from dotenv import load_dotenv
import uvicorn
from STT import speech_to_text
import soundfile as sf

app = FastAPI()
load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

@app.on_event("startup")
def startup_event():
    rclpy.init()
    global node, velocity_publisher
    node = rclpy.create_node('turtlebot_whatsapp_controller')
    velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

@app.on_event("shutdown")
def shutdown_event():
    rclpy.shutdown()

def send_turtlebot_command(linear=0.0, angular=0.0):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    velocity_publisher.publish(vel_msg)

def initiateNav(nav):
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
    return nav

def create_pose_stamped(navigator, pos_x, pos_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.position.z = pos_x
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def modo_autonomo(x, y, z):
    
    nav = BasicNavigator()
    
    if nav.isNav2Active():
        initiateNav(nav)
        nav.waitUntilNav2Active()
    
    goal_pose = create_pose_stamped(nav, x, y, z)

    nav.followWaypoints([goal_pose])

    while not nav.isTaskComplete():
        print(nav.getFeedback())

def navigate_robot(x: float, y: float, z_rotation: float):
    nav = BasicNavigator()
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, z_rotation)

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w

    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w

    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print("Feedback:", feedback)

    print("Navigation task completed.")
    return "Navigation task completed!"

class NavigationCommand(BaseModel):
    x: float
    y: float
    z_rotation: float

@app.post('/bot')
async def bot(Body: str = Form(...), MediaUrl0: str = Form(None)):
    incoming_msg = Body.lower()
    resp = MessagingResponse()
    msg = resp.message()
    responded = False
    strMsg = ""
    message_type = None
    try:
        if MediaUrl0:
            media_content = requests.get(
                MediaUrl0,
                auth=("SID", "TOKEN")
            ).content
            with open("audio_message.ogg", "wb") as f:
                f.write(media_content)
            data, samplerate = sf.read("audio_message.ogg")
            sf.write("audio.wav", data, samplerate)
            strMsg = speech_to_text("audio.wav")
            message_type = "audio"
        else:
            message_type = "text"
            strMsg = incoming_msg
        print(strMsg)

    except Exception as e:
        print(f"Error processing media: {e}")
        message_type = "unknown"

    if not responded:
        response = client.chat.completions.create(
            model="gpt-4",
            messages=[
                {
                    "role": "system",
                    "content": (
                        "Você é um assistente de controle de robô. Interprete a entrada do usuário e decida "
                        "quando acionar a função 'navigate_robot'."
                    ),
                },
                {"role": "user", "content": incoming_msg},
            ],
            functions=[
                {
                    "name": "navigate_robot",
                    "description": "Mova o robô para uma localização específica com uma rotação dada.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number", "description": "Coordenada X da posição de destino."},
                            "y": {"type": "number", "description": "Coordenada Y da posição de destino."},
                            "z_rotation": {
                                "type": "number",
                                "description": "Rotação em torno do eixo Z em radianos."
                            },
                        },
                        "required": ["x", "y", "z_rotation"],
                    },
                }
            ],
        )

        if response.choices[0].message.function_call:
            function_call = response.choices[0].message.function_call
            if function_call.name == "navigate_robot":
                try:
                    args = json.loads(function_call.arguments)
                    x = args["x"]
                    y = args["y"]
                    z_rotation = args["z_rotation"]

                    result = modo_autonomo(x, y, z_rotation)
                    print(result)
                except (json.JSONDecodeError, KeyError) as e:
                    print(f"Error parsing arguments: {e}")
        else:
            print(response.choices[0].message.content)

        msg.body(response.choices[0].message.content)

    return Response(content=str(resp), media_type="application/xml")

if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5000)
