import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)


class UserInterface:
    def __init__(self):
        self.sport_client = SportClient()  
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

    def move(self, vx, vy, vyaw):
        self.sport_client.Move(vx, vy, vyaw)

    def stand_up(self):
        self.sport_client.StandUp()

    def stand_down(self):
        self.sport_client.StandDown()

    def move_forward(self):
        self.sport_client.Move(0.3, 0, 0)

    def move_lateral(self):
        self.sport_client.Move(0, 0.3, 0)

    def move_rotate(self):
        self.sport_client.Move(0, 0, 0.5)

    def stop_move(self):
        self.sport_client.StopMove()

    def switch_gait_0(self):
        self.sport_client.SwitchGait(0)

    def switch_gait_1(self):
        self.sport_client.SwitchGait(1)

    def balance_stand(self):
        self.sport_client.BalanceStand()

    def recovery_stand(self):
        self.sport_client.RecoveryStand()

    def left_flip(self):
        ret = self.sport_client.LeftFlip()
        print("ret: ", ret)

    def back_flip(self):
        ret = self.sport_client.BackFlip()
        print("ret: ", ret)

    def free_walk(self):
        ret = self.sport_client.FreeWalk(True)
        print("ret: ", ret)

    def free_bound(self):
        ret = self.sport_client.FreeBound(True)
        print("ret: ", ret)
        time.sleep(2)
        ret = self.sport_client.FreeBound(False)
        print("ret: ", ret)

    def free_avoid(self):
        ret = self.sport_client.FreeAvoid(True)
        print("ret: ", ret)
        time.sleep(2)
        ret = self.sport_client.FreeAvoid(False)
        print("ret: ", ret)

    def walk_stair(self):
        ret = self.sport_client.WalkStair(True)
        print("ret: ", ret)
        time.sleep(10)
        ret = self.sport_client.WalkStair(False)
        print("ret: ", ret)

    def walk_upright(self):
        ret = self.sport_client.WalkUpright(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = self.sport_client.WalkUpright(False)
        print("ret: ", ret)

    def cross_step(self):
        ret = self.sport_client.CrossStep(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = self.sport_client.CrossStep(False)
        print("ret: ", ret)

    def free_jump(self):
        ret = self.sport_client.FreeJump(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = self.sport_client.FreeJump(False)
        print("ret: ", ret)

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, sys.argv[1])

    test_option = TestOption(name=None, id=None) 
    user_interface = UserInterface()
    user_interface.test_option_ = test_option

    sport_client = SportClient()  
    sport_client.SetTimeout(10.0)
    sport_client.Init()
    while True:

        user_interface.terminal_handle()

        print(f"Updated Test Option: Name = {test_option.name}, ID = {test_option.id}\n")

        if test_option.id == 0:
            sport_client.Damp()
        elif test_option.id == 1:
            sport_client.StandUp()
        elif test_option.id == 2:
            sport_client.StandDown()
        elif test_option.id == 3:
            sport_client.Move(0.3,0,0)
        elif test_option.id == 4:
            sport_client.Move(0,0.3,0)
        elif test_option.id == 5:
            sport_client.Move(0,0,0.5)
        elif test_option.id == 6:
            sport_client.StopMove()
        elif test_option.id == 7:
            sport_client.SwitchGait(0)
        elif test_option.id == 8:
            sport_client.SwitchGait(1)
        elif test_option.id == 9:
            sport_client.BalanceStand()
        elif test_option.id == 10:
            sport_client.RecoveryStand()
        elif test_option.id == 11:
            ret = sport_client.LeftFlip()
            print("ret: ",ret)
        elif test_option.id == 12:
            ret = sport_client.BackFlip()
            print("ret: ",ret)
        elif test_option.id == 13:
            ret = sport_client.FreeWalk(True)
            print("ret: ",ret)
        elif test_option.id == 14:
            ret = sport_client.FreeBound(True)
            print("ret: ",ret)
            time.sleep(2)
            ret = sport_client.FreeBound(False)
            print("ret: ",ret)
        elif test_option.id == 14:
            ret = sport_client.FreeBound(True)
            print("ret: ",ret)
            time.sleep(2)
            ret = sport_client.FreeBound(False)
            print("ret: ",ret)
        elif test_option.id == 15:
            ret = sport_client.FreeAvoid(True)
            print("ret: ",ret)
            time.sleep(2)
            ret = sport_client.FreeAvoid(False)
            print("ret: ",ret)
        elif test_option.id == 16:
            ret = sport_client.WalkStair(True)
            print("ret: ",ret)
            time.sleep(10)
            ret = sport_client.WalkStair(False)
            print("ret: ",ret)
        elif test_option.id == 17:
            ret = sport_client.WalkUpright(True)
            print("ret: ",ret)
            time.sleep(4)
            ret = sport_client.WalkUpright(False)
            print("ret: ",ret)
        elif test_option.id == 18:
            ret = sport_client.CrossStep(True)
            print("ret: ",ret)
            time.sleep(4)
            ret = sport_client.CrossStep(False)
            print("ret: ",ret)
        elif test_option.id == 19:
            ret = sport_client.FreeJump(True)
            print("ret: ",ret)
            time.sleep(4)
            ret = sport_client.FreeJump(False)
            print("ret: ",ret)

        time.sleep(1)
