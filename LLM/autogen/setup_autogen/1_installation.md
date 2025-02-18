AutoGen目前依然处理快速发展的阶段，社区非常活跃，大约每隔两周就会有新版本出来，当前这个教程使用的是0.3.1版本，大家安装的时候要指定版本。

单独安装时使用以下命令：

```sh
pip install autogen==0.3.1 --index-url https://mirrors.aliyun.com/pypi/simple
```

推荐一次性安装大部分依赖，这样可以减少后面因为依赖问题浪费时间

```sh
pip install -r requirements.txt --index-url https://mirrors.aliyun.com/pypi/simple
```

如果在安装过程中，提示依赖的版本不适配，可以把该依赖的版本号从requirements.txt去掉，
或者先跳过该依赖，等到提示缺少该依赖时，再手动安装。



openai.BadRequestError: Error code: 400 - {'error': {'message': 'Provider API error: Stream not yet supported with tool calls (request id: 20250217150358353950878v4Pzkeaj)', 'param': 'stream', 'type': 'invalid_request_error'}}


openai.BadRequestError: Error code: 400 - {'error': {'message': 'registry.ollama.ai/library/deepseek-r1:7b does not support tools', 'type': 'api_error', 'param': None, 'code': None}}