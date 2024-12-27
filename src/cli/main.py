import click
import inquirer
import paramiko
import threading
import time
import subprocess
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from math import pi
from utils import landmarks, create_pose_stamped

global nav

ssh = None
bringup_thread = None

def print_banner():
    click.clear()
    banner = """
████████╗██╗   ██╗██████╗ ████████╗██╗     ███████╗██████╗  ██████╗ ████████╗
╚══██╔══╝██║   ██║██╔══██╗╚══██╔══╝██║     ██╔════╝██╔══██╗██╔═══██╗╚══██╔══╝
   ██║   ██║   ██║██████╔╝   ██║   ██║     █████╗  ██████╔╝██║   ██║   ██║   
   ██║   ██║   ██║██╔══██╗   ██║   ██║     ██╔══╝  ██╔══██╗██║   ██║   ██║   
   ██║   ╚██████╔╝██║  ██║   ██║   ███████╗███████╗██████╔╝╚██████╔╝   ██║   
   ╚═╝    ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚══════╝╚═════╝  ╚═════╝    ╚═╝   
                       TURTLEBOT CONTROL CLI
    """
    click.secho(banner, fg="cyan", bold=True)
    click.secho(
        "Controle o TurtleBot de forma simples e intuitiva!", fg="yellow", bold=True
    )
    click.echo("")


def loading_animation(text, duration=2):
    click.secho(text, nl=False, fg="magenta")
    for _ in range(duration):
        click.secho(".", nl=False, fg="magenta")
        time.sleep(0.5)
    click.echo("")


def run_bringup():
    global ssh
    try:
        bringup_command = "source /opt/ros/humble/setup.bash && export LDS_MODEL=LDS-02 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=109 && export TURTLEBOT3_MODEL=burger && source ~/turtlebot3_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 launch turtlebot3_bringup robot.launch.py"
        stdin, stdout, stderr =  ssh.exec_command(bringup_command, get_pty=True)
        click.echo("Executando bringup do TurtleBot...")

        for line in iter(stdout.readline, ""):
            click.echo(line.strip())

    except Exception as e:
        click.secho(f"Erro ao executar o bringup: {e}", fg="red", bold=True)


def conectar_robo():
    """Conectar o TurtleBot via SSH and start bringup in a separate thread"""
    global ssh, bringup_thread
    click.secho("\n>>> Tentando conectar ao TurtleBot via SSH...", fg="blue", bold=True)

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(hostname="grupo2.local", username="grupo2", password="Repipe2")
        loading_animation("Conectando ao robô", duration=3)
        click.secho("Conexão estabelecida com sucesso! 🎉", fg="green", bold=True)

        bringup_thread = threading.Thread(target=run_bringup)
        bringup_thread.daemon = True
        bringup_thread.start()

    except Exception as e:
        click.secho(f"Erro ao conectar: {e}", fg="red", bold=True)

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

def teleoperar_robo():
    click.secho("\n>>> Iniciando teleoperação do TurtleBot...", fg="blue", bold=True)
    loading_animation("Iniciando teleop", duration=2)

    click.secho(
        "Use as teclas de direção para mover o TurtleBot", fg="yellow", bold=True
    )

    try:
        # Run the teleop command as a subprocess to allow real-time keyboard control
        subprocess.run("ros2 run turtlebot3_teleop teleop_keyboard", shell=True)
    except Exception as e:
        click.secho(f"Erro ao iniciar teleoperação: {e}", fg="red", bold=True)

    click.secho("Teleop encerrado.", fg="green")


def mapear_ambiente():
    click.secho("\n>>> Iniciando mapeamento do ambiente atual...", fg="blue", bold=True)
    loading_animation("Mapeando o ambiente", duration=2)

    click.secho("Mapeando... (simulado)", fg="blue", bold=True)
    time.sleep(5)
    click.secho("Mapa salvo com sucesso! 🗺️", fg="green", bold=True)


def modo_autonomo(nav):
    
    ambientes = ["Home", "Ateliê 9", "Laboratório", "Sala de Aula"]
    perguntas = [
        inquirer.List(
            "ambiente",
            message="Escolha o ambiente para navegação autônoma",
            choices=ambientes,
        )
    ]
    respostas = inquirer.prompt(perguntas)

    if respostas:
        ambiente = respostas["ambiente"]
        selectedPosition = ambientes.index(respostas["ambiente"])

        # TODO Implement thread

        # nav_command = "ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/josevalencar/inteli/m8/2024-2B-T08-EC08-G01/src/cli/map/lab.yaml"
        # subprocess.run(nav_command, shell=True)

        goal_pose = create_pose_stamped(nav, landmarks[selectedPosition][0], landmarks[selectedPosition][1], landmarks[selectedPosition][2])

        click.secho(
            f"\n>>> Iniciando execução autônoma no ambiente: {ambiente}",
            fg="blue",
            bold=True,
        )

        nav.followWaypoints([goal_pose])

        while not nav.isTaskComplete():
            click.secho(
                f"Movendo-se de forma autônoma no ambiente {ambiente}... 🚗",
                fg="green",
                bold=True,
            )
            time.sleep(5)
        click.secho("Execução autônoma concluída.", fg="green", bold=True)


def encerrar_comunicacao():
    global ssh
    click.secho("\n>>> Encerrando comunicação com o robô...", fg="blue", bold=True)
    if ssh:
        ssh.close()
        click.secho("Comunicação com o robô encerrada. 🔴", fg="red", bold=True)
    else:
        click.secho("Nenhuma conexão ativa para encerrar.", fg="yellow", bold=True)
    rclpy.shutdown()

def menu_principal():
    print_banner()
    rclpy.init()
    nav = BasicNavigator()
    while True:
        perguntas = [
            inquirer.List(
                "acao",
                message=" ",
                choices=[
                    "Conectar ao robô",
                    "Iniciar modo autônomo",
                    "Teleoperar o robô",
                    "Mapear ambiente e salvar mapa",
                    "Modo autônomo",
                    "Encerrar comunicação com o robô",
                    "Sair",
                ],
            )
        ]
        resposta = inquirer.prompt(perguntas)

        if resposta:
            acao = resposta["acao"]

            if acao == "Conectar ao robô":
                conectar_robo()
            elif acao == "Iniciar modo autônomo":
                nav = initiateNav(nav)
            elif acao == "Teleoperar o robô":
                teleoperar_robo()
            elif acao == "Mapear ambiente e salvar mapa":
                mapear_ambiente()
            elif acao == "Modo autônomo":
                modo_autonomo(nav)
            elif acao == "Encerrar comunicação com o robô":
                encerrar_comunicacao()
            elif acao == "Sair":
                click.secho("\nSaindo da CLI TurtleBot... 👋", fg="cyan", bold=True)
                break

        click.pause(info="Pressione Enter para voltar ao menu principal...")


if __name__ == "__main__":
    menu_principal()
