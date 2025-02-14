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
            'content': '\n请写一首歌颂赵灵儿的四言绝句',
        }
    ],
    model='deepseek-r1:32b',
    temperature = 0.7,                               # 控制生成的随机性
    # max_tokens = 256,                                # 生成的最大token数
)

# print("deepseek回复如下：")
print(chat_completion.choices[0].message.content)