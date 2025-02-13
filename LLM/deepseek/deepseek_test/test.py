from openai import OpenAI

client = OpenAI(
    base_url='http://localhost:11434/v1/', # ollama的API地址

    # required but ignored
    api_key='ollama',
)

chat_completion = client.chat.completions.create(
    messages=[
        {
            'role': 'user',
            'content': '\n你好，帮我写一段ros2框架下的python代码，可以实现调用机器人的摄像头接口识别环境中是否有苹果',
        }
    ],
    model='deepseek-r1:7b',
    temperature = 0.7,                               # 控制生成的随机性
    # max_tokens = 256,                                # 生成的最大token数
)

# print("deepseek回复如下：")
print(chat_completion.choices[0].message.content)