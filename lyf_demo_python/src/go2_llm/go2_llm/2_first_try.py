"""
在运行代码之前，需要先设置一个环境变量,
AUTOGEN_USE_DOCKER=False
"""
import os
from autogen import AssistantAgent, UserProxyAgent
from dotenv import load_dotenv

load_dotenv()  # 加载 .env 文件中的环境变量

llm_config = {
    "model": os.environ.get("OPENAI_MODEL"),           # 还有一个模型是转为写代码开发的deepseek-coder
    "api_key": os.environ.get("OPENAI_API_KEY"),
    "base_url": os.environ.get("OPENAI_BASE_URL"),    # 如果报错，改成"https://api.deepseek.com/v1"


    # "model": os.getenv("OPENAI_MODEL"),
    # "api_key": os.getenv("OPENAI_API_KEY"),
    # "base_url": os.getenv("OPENAI_BASE_URL"),


    "temperature": 0.8,
}

assistant = AssistantAgent("assistant", llm_config=llm_config)      # 传入大模型的配置

user_proxy = UserProxyAgent("user_proxy", code_execution_config=False)      # 不需要执行code

# Start the chat
user_proxy.initiate_chat(
    assistant,
    message="详细介绍一下ROS2",
)