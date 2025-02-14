import os
import autogen
from autogen.coding import LocalCommandLineCodeExecutor
from autogen import register_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import time
import cv2
import os
from datetime import datetime
from typing import Optional
import uuid

from dotenv import load_dotenv


from typing import Any
from unitree_go.msg import SportModeState                           # 导入SportModeState消息类型，订阅状态信息
from unitree_api.msg import Request                                 # 导入Request消息类型，发送运动指令
from go2_sport.ros2_sport_client import (SportClient,PathPoint)     # 高层运动控制接口API调用 

SPORTTOPIC = "api/sport/request"                                     # 添加运动控制话题

load_dotenv()  # 加载 .env 文件中的环境变量

# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色


# 类型提示风格
def write_to_file(content: Any, file_path: str) -> None:
    """
    Write the given content to a specified file path.

    Args:
        content (Any): The content to be written into the file. This can be of any type
                       that can be converted to string.
        file_path (str): The path of the file where the content will be written.

    Returns:
        None
    """
    try:
        # Ensure the directory of file_path exists, if not create it
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        # Open the file in write mode. If the file does not exist, it will be created.
        with open(file_path, "w", encoding="utf-8") as file:
            # Convert the content to string and write it to the file.
            file.write(str(content))
    except IOError as e:
        # Handle any I/O error that occurs
        print(f"An error occurred while writing to the file: {e}")
    except Exception as e:
        # Handle any other unexpected errors
        print(f"An unexpected error occurred: {e}")


# def capture_and_save_image() -> Optional[str]:
#     """
#     Capture an image from the camera and save it to a specified directory.

#     This function opens the camera, captures a single frame, and saves it as a JPEG file
#     with a timestamp in the filename. The image is saved in the '/home/flyivan/learn_ros/guyueclass/embodied_intelligence/multi_agent_autogen/autogen_code/6_robotic_llm' directory.

#     Returns:
#         Optional[str]: The full path of the saved image file if successful, None otherwise.
#     """
#     # Open the camera
#     # Note: 8 is used as the camera index, adjust if necessary
#     cap: cv2.VideoCapture = cv2.VideoCapture(8)

#     if not cap.isOpened():
#         print("Unable to open camera")
#         return None

#     # Capture a single frame
#     ret: bool
#     frame: cv2.Mat
#     ret, frame = cap.read()

#     if not ret:
#         print("Failed to capture image")
#         cap.release()
#         return None

#     # Create save directory
#     save_path: str = '/home/flyivan/learn_ros/guyueclass/embodied_intelligence/multi_agent_autogen/autogen_code/6_robotic_llm'
#     os.makedirs(save_path, exist_ok=True)

#     # Generate filename with timestamp
#     timestamp: str = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
#     filename: str = f"{timestamp}.jpg"
#     full_path: str = os.path.join(save_path, filename)

#     # Save the image
#     cv2.imwrite(full_path, frame)
#     print(f'Image saved to {full_path}')

#     # Release the camera
#     cap.release()

#     return full_path


class RobotMover(Node):
    """
    A ROS2 node for controlling robot movement.
    
    This class publishes Twist messages to control the linear and angular velocity of a robot.
    """

    def __init__(self) -> None:
        """
        Initialize the RobotMover node with a unique name and create a publisher for velocity commands.
        """
        # Use a unique node name
        super().__init__(f'robot_mover_{uuid.uuid4().hex}')

        # 创建go2控制指令发布者
        self.cmd_vel_pub: rclpy.publisher.Publisher = self.create_publisher(Request, SPORTTOPIC, 10)       # 创建运动控制的发布者 



    def move_forward(self, distance: float) -> None:
        """
        Move the robot forward for a specified distance.Using the dedicated API for robots

        Args:
            distance (float): The distance to move forward in meters.
        """
        req: Request = Request()
        sport_req: SportClient = SportClient()
        vx = 0.2
        vy = 0.0
        vyaw = 0.0
        sport_req.Move(req,vx, vy, vyaw)

        time_to_move: float = distance / 0.2  # Calculate time needed to move the specified distance

        # Ensure the message is received by publishing multiple times
        for _ in range(10):
            self.cmd_vel_pub.publish(req)
            time.sleep(0.1)

        print(f'x={vx}, y={vy}, vyaw={vyaw}')

        time.sleep(time_to_move)  # Wait for the calculated time

        # Stop the robot
        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        sport_req.Move(req,vx, vy, vyaw)
        for _ in range(10):
            self.cmd_vel_pub.publish(req)
            time.sleep(0.1)


def move_forward(distance: float) -> str:
    """
    Initialize ROS2, create a RobotMover node, move the robot, and then shut down.

    Args:
        distance (float): The distance for the robot to move forward in meters.
    """
    rclpy.init()
    mover: RobotMover = RobotMover()
    mover.move_forward(distance)
    mover.destroy_node()
    rclpy.shutdown()
    return f"Robot moved forward {distance} meters."
    

go2bot_system_prompt = """
go2是一个开源的四足机器人，有一个摄像头在它的正前方，
基于ros2 foxy开发，以下是go2提供的高层运控接口：
1. Move()接口，起到速度控制作用，可控制x\y轴线速度和yaw的角速度，接口类型是SportClient,定义如下：
        def Move(self, req: Request, vx: float, vy: float, vyaw: float):
            js = {
                "x": vx,
                "y": vy,
                "z": vyaw
            }
            req.parameter = json.dumps(js)
            req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE

"""

llm_config_gpt4o = {
    "model": os.getenv("OPENAI_MODEL"),
    "api_key": os.getenv("OPENAI_API_KEY"),
    "base_url": os.getenv("OPENAI_BASE_URL"),
    # "api_type": "azure",
    # "api_version": "2024-02-15-preview",
    "temperature": 0.8,
    "stream": True,
}

initializer = autogen.UserProxyAgent(
    name="Init",
)

robotic_expert = autogen.AssistantAgent(
    name="robotic_expert",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名机器人开发专家，
    熟悉Python，ROS2， SLAM，vSLAM，Navigation2，运动规划和控制，RTOS，AI，嵌入式开发等技术栈，""" +
    go2bot_system_prompt + """
    需要写代码时，尽量使用Python和shell，
    如果有不清楚的地方，你要根据已有的信息进行合理的推测，但是要明确地说明这是你的推测，
    你写好的代码和命令最终会被code_executor执行，写代码时要考虑到便于他执行
    """,
    description="我是机器人开发专家，robotic_expert，"
)

human_proxy = autogen.AssistantAgent(
    name="human_proxy",
    llm_config=False,  # no LLM used for human proxy
    human_input_mode="ALWAYS",  # always ask for human input
    description="我是老板，你们要完成我交代的任务",
)


code_copy = autogen.AssistantAgent(
    name="code_copy",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名机器人开发专家，
    你只需要调用write_to_file这个工具把robotic_expert的代码写入到本地的/home/flyivan/robot路径下，
    并且考虑到相关技术栈的限制和约定，
    如果代码有改动，你也要及时更新文件
    """,
    description="我叫code_copy，我可以调用write_to_file这个tool把robotic_expert的所有代码合理的写入到本地，每一次robotic_expert输出代码之后呼叫我"
)


executor = LocalCommandLineCodeExecutor(
    timeout=60,  # Timeout for each code execution in seconds.
    work_dir="/tmp",  # Use the temporary directory to store the code files.
)

code_executor = autogen.AssistantAgent(
    "code_executor",
    llm_config=False,  # Turn off LLM for this agent.
    code_execution_config={
        "executor": executor
    },  # Use the local command line code executor.
    human_input_mode="NEVER",  # Always take human input for this agent for safety.
    description="我可以执行其他Agents写好的代码，需要执行代码的时候呼叫我",
)

# register_function(
#     capture_and_save_image,
#     caller=robotic_expert,  # The assistant agent can suggest calls to the calculator.
#     executor=code_executor,  # The user proxy agent can execute the calculator calls.
#     name="capture_and_save_image",  # By default, the function name is used as the tool name.
#     description="从go2摄像头获取一张照片并保存在本地，并返回图片的绝对路径",  # A description of the tool.
# )

register_function(
    move_forward,
    caller=robotic_expert,  # The assistant agent can suggest calls to the calculator.
    executor=code_executor,  # The user proxy agent can execute the calculator calls.
    name="move_forward",  # By default, the function name is used as the tool name.
    description="让go2向前移动一段距离",  # A description of the tool.
)

register_function(
    write_to_file,
    caller=code_copy,
    executor=code_executor,
    name="write_to_file",
    description="把指定的内容写入一下指定的路径",
)


graph_dict = {}
graph_dict[initializer] = [robotic_expert]
graph_dict[human_proxy] = [robotic_expert]
graph_dict[robotic_expert] = [code_copy, code_executor]
graph_dict[code_copy] = [code_executor]
graph_dict[code_executor] = [human_proxy]

agents = [initializer, human_proxy, robotic_expert, code_executor, code_copy]


groupchat = autogen.GroupChat(
    agents=agents,
    messages=[],
    max_round=20,
    allowed_or_disallowed_speaker_transitions=graph_dict,
    speaker_transitions_type="allowed"
)
manager = autogen.GroupChatManager(groupchat=groupchat, llm_config=llm_config_gpt4o)

initializer.initiate_chat(
    manager,
    message="向前走一段距离",
    clear_history=False     # 不能清除历史记忆
)

