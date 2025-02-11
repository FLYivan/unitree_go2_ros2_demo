# 一、本地部署步骤

    1、从 https://ollama.com/ 官网中选择linux版，按如下指令下载ollama

        curl -fsSL https://ollama.com/install.sh | sh

    2、在ollama的官网上，选择"Models"，选择deepseek-r1 32b，按如下指令下载安装

        ollama run deepseek-r1:32b
    
    3、安装openai库，以便通过api调用

    