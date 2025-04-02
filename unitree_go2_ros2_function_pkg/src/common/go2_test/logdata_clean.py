import re
import math

class PathData :
    def __init__(self):
        self.results = []                                                # 用于存储解析结果的列表        

    def parse_log_file(self,file_path, keyword):

        pattern = r'最新前方障碍距离：([\d\.]+|inf),.*?最新左侧障碍距离：([\d\.]+|inf),.*?最新右侧障碍距离：([\d\.]+|inf)(?=\x1b\[0m)'        # 提取数字或inf，同时舍弃
        # (?=\x1b\[0m) 是一个正向前瞻（lookahead），用于确保匹配的数值后面紧跟着 \x1b[0m，但不包括在提取的结果中。

        with open(file_path, 'r', errors='ignore') as file:                          # 以只读模式打开指定路径的日志文件
            for line in file:                                       # 逐行读取文件内容
                if  keyword in line:                                # 如果当前行包含指定的关键词                
                    # 使用正则表达式直接提取前，左，右值
                    match = re.search(pattern, line)
                    if match:

                        self.results.append([
                            float(match.group(1)),
                            float(match.group(2)),
                            float(match.group(3))
                        ]
                        )  # 将解析结果存入列表

    def get_all_distances(self):
        # 返回所有距离值
        return self.results



def main():
    
    log_file_path = '/home/flyivan/dog_robot/ros2_demo/analy/python3_17129_1729243600468.log'                   # 日志文件路径
    keyword = '最新前方障碍距离'                                                                                   # 日志关键词

    parsed_data = PathData()
    parsed_data.parse_log_file(log_file_path, keyword)                                                          # 调用函数解析日志文件

    # 打印结果
    for distances in parsed_data.get_all_distances():
        front_distance = distances[0]
        left_distance = distances[1]
        right_distance = distances[2]
        b = type(left_distance)

        # 根据需要处理每个距离值
        print(f"前方距离：{front_distance}, 左侧距离：{left_distance}, 右侧距离：{right_distance}, {b}")




if __name__ == '__main__':
    main()                                                                                                      # 如果脚本作为主程序运行，则调用main函数



