import os
import autogen
from autogen import AssistantAgent, UserProxyAgent
from dotenv import load_dotenv

load_dotenv()  # 加载 .env 文件中的环境变量


llm_config = {
    "model": os.getenv("OPENAI_MODEL"),
    "api_key": os.getenv("OPENAI_API_KEY"),
    "base_url": os.getenv("OPENAI_BASE_URL"),
    "temperature": 0.8,
}

assistant = AssistantAgent("assistant", llm_config=llm_config)

user_proxy = UserProxyAgent(
    "user_proxy", code_execution_config={"executor": autogen.coding.LocalCommandLineCodeExecutor(work_dir="coding")}        # 指定工作目录
)

# AutoGen 的代码执行有本地执行和在docker中执行两种，
# 本地执行就是在当前代码运行的系统中执行，
# 在docker执行时，AutoGen会起一个容器，然后在这个容器中运行命令
# 这么做的是出于安全考虑
# AutoGen的开发者认为大模型产生的代码和命令可能存在风险

# Start the chat
user_proxy.initiate_chat(
    assistant,
    message="查看一下当前系统占用资源最多的进程是哪个？",
)