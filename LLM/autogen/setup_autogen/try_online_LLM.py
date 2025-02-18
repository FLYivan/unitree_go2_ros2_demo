import os
import autogen
from autogen import AssistantAgent, UserProxyAgent
from dotenv import load_dotenv

load_dotenv()  # 加载 .env 文件中的环境变量


llm_config = {
    # "model": os.getenv("OPENAI_MODEL"),
    # "api_key": os.getenv("OPENAI_API_KEY"),
    # "base_url": os.getenv("OPENAI_BASE_URL"),

    # "model": os.getenv("UNIPAY_LLM_MODEL"),
    # "api_key": os.getenv("UNIPAY_LLM_API_KEY"),
    # "base_url": os.getenv("UNIPAY_LLM_BASE_URL"),

    "model": os.getenv("LOCAL_LLM_MODEL"),
    "api_key": os.getenv("LOCAL_LLM_API_KEY"),
    "base_url": os.getenv("LOCAL_LLM_BASE_URL"),



    "temperature": 0.8,
}

assistant = AssistantAgent("assistant", llm_config=llm_config)

user_proxy = UserProxyAgent(
    "user_proxy", code_execution_config={"executor": autogen.coding.LocalCommandLineCodeExecutor(work_dir="coding")}        # 指定工作目录
)

# Start the chat
user_proxy.initiate_chat(
    assistant,
    message="查看一下当前系统占用资源最多的进程是谁？",
)