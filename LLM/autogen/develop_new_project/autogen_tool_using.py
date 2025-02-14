import os
import autogen
from autogen.coding import LocalCommandLineCodeExecutor
from autogen import register_function
from typing import Any
from dotenv import load_dotenv

load_dotenv()  # 加载 .env 文件中的环境变量


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


llm_config_gpt4o = {
    "model": os.getenv("OPENAI_MODEL"),
    "api_key": os.getenv("OPENAI_API_KEY"),
    "base_url": os.getenv("OPENAI_BASE_URL"),
    # "api_type": "azure",
    # "api_version": "2024-02-15-preview",
    "temperature": 0.9,
}

initializer = autogen.UserProxyAgent(
    name="Init",
)

frontend_expert = autogen.AssistantAgent(
    name="frontend_expert",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名前端开发工程师，
    擅长CSS，HTML和javascript，
    请你根据PM的规划进行开发，
    写代码时只能使用原生的css、JavaScript和html，不要使用任何框架和组件，
    重要的是，你要遵循这种方法，
    你应避免不必要的道歉，并在回顾对话时不重复早先的错误
    """,
    description="我是前端开发专家",
)

frontend_reviewer = autogen.AssistantAgent(
    name="frontend_reviewer",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名前端开发工程师，
    擅长CSS，HTML和javascript，
    你要review frontend_expert的方案和代码是否有错误，如果指出错误，并给出正确的方案，
    你要在frontend_expert代码的基础上，结合你review的结果，重新给出完整的代码和步骤，
    写代码时只能使用原生的css、JavaScript和html，不要使用任何框架和组件，
    重要的是，你要遵循这种方法，
    你应避免不必要的道歉，并在回顾对话时不重复早先的错误""",
    description="我是frontend code reviewer，会对frontend_expert的输出进行reviewe， \
    frontend_expert的每一次输出，我只会review一次，所以不要连续两次呼叫我",
)

frontend_pm = autogen.AssistantAgent(
    name="frontend_pm",
    llm_config=llm_config_gpt4o,
    system_message="""你是一名前端产品经理，尤其擅长个人主页项目和产品，
    请你尽力理解老板的任务，并重新描述成一个具体的、可执行的任务，以便frontend_expert和frontend_viewer更好地进行开发，
    任务的最终交付物应该是代码、命令、部署步骤等详细的解释/说明等内容，代码和命令是非常重要的,
    项目的根路径是/home/flyivan/autogen-frontend，你要让code_copy调用write_to_file把相关的代码复制进去
    你自己不要写代码，并且其他人代码时只能使用原生的css、JavaScript和html，不要使用任何框架和组件，
    如果有不清楚的地方，你可以根据已有的信息进行合理的推测，但是要明确地说明这是你的推测；
    也可以要求老板补充更多信息
    """,
)

code_copy = autogen.AssistantAgent(
    name="code_copy",
    llm_config=llm_config_gpt4o,
    system_message="""你是一个前端专家，
    你要调用write_to_file这个工具把frontend_expert的代码写入到本地的/home/flyivan/autogen-frontend路径下，
    并且考虑到相关技术栈的限制和约定，
    如果代码有改动，你也要及时更新文件
    """,
    description="我可以调用write_to_file这个tool把frontend_expert的代码合理的写入到本地，每一次frontend_expert输出代码之后呼叫我",
)

human_proxy = autogen.AssistantAgent(
    name="human_proxy",
    llm_config=False, 
    human_input_mode="ALWAYS",
    description="我是老板，你们要完成我交代的任务，同时我也是技术小白，你们要尽量让我理解你们的工作 \
        每当frontend_pm, frontend_expert和frontend_viewer都发言之后，至少要呼叫我一次",
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


graph_dict = {}
graph_dict[initializer] = [frontend_pm]
graph_dict[frontend_pm] = [frontend_expert]
graph_dict[frontend_expert] = [frontend_reviewer]
graph_dict[frontend_reviewer] = [code_copy]
graph_dict[code_copy] = [code_executor]


agents = [
    initializer,
    human_proxy,
    frontend_reviewer,
    frontend_expert,
    frontend_pm,
    code_executor,
    code_copy,
]


groupchat = autogen.GroupChat(
    agents=agents,
    messages=[],
    max_round=20,
    allowed_or_disallowed_speaker_transitions=graph_dict,
    speaker_transitions_type="allowed",
)
manager = autogen.GroupChatManager(groupchat=groupchat, llm_config=llm_config_gpt4o)

initializer.initiate_chat(
    manager,
    message="为软件工程师设计一个静态的个人主页，要酷炫一点",
    clear_history=False,
)
