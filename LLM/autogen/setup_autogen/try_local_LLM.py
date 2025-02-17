"""
在运行代码之前，需要先设置一个环境变量,
export AUTOGEN_USE_DOCKER=False
"""
import os
from autogen import AssistantAgent, UserProxyAgent
from dotenv import load_dotenv

load_dotenv()  # 加载 .env 文件中的环境变量

llm_config = {

    # "model": os.getenv("LOCAL_LLM_MODEL"),
    # "api_key": os.getenv("LOCAL_LLM_API_KEY"),
    # "base_url": os.getenv("LOCAL_LLM_BASE_URL"),


    "model": os.getenv("OPENAI_MODEL"),
    "api_key": os.getenv("OPENAI_API_KEY"),
    "base_url": os.getenv("OPENAI_BASE_URL"),


    "temperature": 0.8,
}

assistant = AssistantAgent("assistant", llm_config=llm_config)      # 传入大模型的配置

user_proxy = UserProxyAgent("user_proxy", code_execution_config=False)      # 不需要执行code

# Start the chat
user_proxy.initiate_chat(
    assistant,
    message="中医里说的“望”和“闻”指的是什么",
)