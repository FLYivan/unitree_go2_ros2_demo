import os
import autogen
from dotenv import load_dotenv
from typing import Any
from autogen.coding import LocalCommandLineCodeExecutor
from autogen import register_function

load_dotenv()  # 加载 .env 文件中的环境变量

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



yolo_expert = autogen.AssistantAgent(
    name="script_expert",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名以中文为母语的脚本软件开发专家，尤其擅长Python
    在解决问题时，你应当将任务分解成小的问题，然后逐个解决，
    需要写代码时，使用Python和Shell,并且要尽量完善、可用，要有详细的步骤、命令和完整的代码,
    如果有不清楚的地方，你要根据已有的信息进行合理的推测，但是要明确地说明这是你的推测，
    在解决问题的每一个步骤中，你都要给出必要且详细的说明，以便其他人可以充分理解你的工作，
    重要的是，你要遵循这种方法，并尽最大努力教会你的对话者如何做出有效的决策。
    你应避免不必要的道歉，并在回顾对话时不重复早先的错误""",
    description="我叫script_expert，是脚本软件开发专家，每当script_pm设计好任务之后应该呼叫我进行开发"
)

yolo_reviewer = autogen.AssistantAgent(
    name="script_reviewer",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名以中文为母语的脚本软件开发专家，尤其擅长Python
    你要review script_expert的方案和代码是否有错误，尤其是script_expert是否给出了完整的Python代码，如果指出错误，并给出正确的方案，
    你不要自己直接设计方案或者写代码，只需要review script_expert的输出即可
    重要的是，你要遵循这种方法，
    你应避免不必要的道歉，并在回顾对话时不重复早先的错误""",
    description="我叫script_reviewer，会对script_expert的输出进行review"             # 都是以“我叫XX”开头
)

yolo_pm = autogen.AssistantAgent(
    name="script_pm",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名以中文为母语的
    脚本软件的产品经理，尤其擅长游戏外挂脚本的产品和项目，
    请你尽力理解老板的任务，并重新描述成一个具体的、可执行的任务，以便script_expert和script_reviewer更好地进行开发，
    任务的最终交付物应该是代码、命令、部署步骤等详细的解释/说明等内容，代码和命令是非常重要的,
    项目的根路径是/home/flyivan/script，你要让code_copy调用write_to_file把相关的代码复制进去
    你自己不要写代码
    如果有不清楚的地方，你可以根据已有的信息进行合理的推测，但是要明确地说明这是你的推测；
    也可以要求老板补充更多信息
    """,
    description="我叫script_pm，是脚本软件的产品经理，我会从简单的需求开始做详细的分析和规划，在需要的时候呼叫我"
)

human_proxy = autogen.AssistantAgent(
    name="human_proxy",                     
    llm_config=False,  # no LLM used for human proxy            因为是人类
    human_input_mode="ALWAYS",  # always ask for human input
    description="我是老板，你们要完成我交代的任务，同时我也是技术小白，你们要尽量让我理解你们的工作 \
        每当script_pm, script_expert和script_reviewer都发过言之后，至少要呼叫我一次",
)


code_copy = autogen.AssistantAgent(
    name="code_copy",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名以中文为母语的脚本软件开发领域的专家，
    你只需要调用write_to_file这个工具把script_expert的代码写入到本地的/home/flyivan/script路径下，
    并且考虑到相关技术栈的限制和约定，
    如果代码有改动，你也要及时更新文件
    """,
    description="我叫code_copy，我可以调用write_to_file这个tool把script_expert的所有代码合理的写入到本地，每一次script_expert输出代码之后呼叫我"
)

executor = LocalCommandLineCodeExecutor(
    timeout=10,
    work_dir="coding",
)

code_executor = autogen.AssistantAgent(
    "code_executor",
    llm_config=False, 
    code_execution_config={
        "executor": executor
    }, 
    human_input_mode="NEVER", 
    description="我可以执行其他Agents写好的代码，需要执行代码的时候呼叫我",
)

register_function(
    write_to_file,
    caller=code_copy,
    executor=code_executor,
    name="write_to_file",
    description="把指定的内容写入一下指定的路径",
)


# 定义群聊的顺序逻辑

graph_dict = {}
graph_dict[initializer] = [yolo_pm]
graph_dict[yolo_pm] = [yolo_expert]
graph_dict[human_proxy] = [yolo_pm, yolo_expert, yolo_reviewer]         # 有多个agent要发言时，description就发挥作用了
graph_dict[yolo_reviewer] = [yolo_expert,code_copy]
graph_dict[yolo_expert] = [human_proxy, yolo_reviewer]
graph_dict[code_copy] = [code_executor]

agents = [initializer, human_proxy, yolo_reviewer, yolo_expert, yolo_pm, code_copy, code_executor]


groupchat = autogen.GroupChat(
    agents=agents,
    messages=[],
    max_round=12,
    allowed_or_disallowed_speaker_transitions=graph_dict,               # 固定参数
    speaker_transitions_type="allowed",                                 # 固定参数
)

# 由llm_config_gpt4o大模型驱动的agent根据description和历史会话记录，判断谁来发言
manager = autogen.GroupChatManager(groupchat=groupchat, llm_config=llm_config_gpt4o)            

initializer.initiate_chat(
    manager,
    message="为移动机器人开发一个目标检测功能，要基于YOLO v5开发，从部署到应用，搭建识别环境、训练自己的模型、应用并提取识别信息, 并给出完整的代码",
    clear_history=False                 # 不清理历史记录
)
