from flask import Flask, request
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

app = Flask(__name__)

# Initialize ROS 2 client
rclpy.init()

# Create a ROS 2 node for publishing commands to TurtleBot3
node = rclpy.create_node('turtlebot_whatsapp_controller')
velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

load_dotenv()

# Instantiate OpenAI client
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Function to send movement commands to TurtleBot3
def send_turtlebot_command(linear=0.0, angular=0.0):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    velocity_publisher.publish(vel_msg)

def navigate_robot(x: float, y: float, z_rotation: float):
    rclpy.init()
    nav = BasicNavigator()

    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, z_rotation)

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
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
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w

    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print("Feedback:", feedback)

    print("Navigation task completed.")
    rclpy.shutdown()
    return "Navigation task completed!"

class NavigationCommand(BaseModel):
    x: float
    y: float
    z_rotation: float

# Route to handle Twilio messages
@app.route('/bot', methods=['POST'])
def bot():
    incoming_msg = request.values.get('Body', '').lower()
    resp = MessagingResponse()
    msg = resp.message()
    responded = False

    # Save voice message audio
    media_url = request.values.get('MediaUrl0', '')
    if media_url:
        media_content = requests.get(
            media_url,
            auth=(os.getenv("TWILIO_ACCOUNT_SID"), os.getenv("TWILIO_AUTH_TOKEN"))
        ).content
        with open("audio_message.ogg", "wb") as f:
            f.write(media_content)

    if not responded:
        # Send message to ChatGPT
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

                    result = navigate_robot(x, y, z_rotation)
                    print(result)
                except (json.JSONDecodeError, KeyError) as e:
                    print(f"Error parsing arguments: {e}")
        else:
            print(response.choices[0].message.content)

        msg.body(response.choices[0].message.content)

    return str(resp)


if __name__ == '__main__':
    try:
        app.run()
    finally:
        rclpy.shutdown()