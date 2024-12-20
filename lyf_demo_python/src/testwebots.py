from controller import Robot, DistanceSensor, Motor
TIME_STEP = 64
# 初始化机器人控制器

robot = Robot()
# 获取传感器和电机
 
sensors = [robot.getDevice(f"ds{i}") for i in range(2)]
for sensor in sensors: 
    sensor.enable(TIME_STEP)

left_motor = robot.getDevice("left_motor")
right_motor = robot.getDevice("right_motor")

# 设置电机速度模式
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# 主循环
while robot.step(TIME_STEP) != -1: 
    left_speed = 3.0
    right_speed = 3.0
    # 获取传感器值   
    sensor_values = [sensor.getValue() for sensor in sensors]

    # 简单避障逻辑   
    if sensor_values[0] > 80.0: # 左侧障碍         
        left_speed = -3.0
    if sensor_values[1] > 80.0: # 右侧障碍       
        right_speed = -3.0
    # 设置电机速度   
    left_motor.setVelocity(left_speed) 
    right_motor.setVelocity(right_speed)