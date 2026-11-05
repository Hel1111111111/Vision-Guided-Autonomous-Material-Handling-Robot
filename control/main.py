# main.py
# 主车运动程序
# 功能：
# 1. 主车不使用摄像头
# 2. 主车只负责低速运动
# 3. 红外灯板保持常亮，让从车通过摄像头跟随
#
# 坐标方向基于你之前测试结果：
# vx > 0：后退
# vx < 0：前进
# vy > 0：向左
# vy < 0：向右
# w  > 0：顺时针
# w  < 0：逆时针

import time
from machine import Pin
from smartcar import ticker
from system import CarSystem
from guide import GuideController
from actions import ActionController
from config import TaskConfig, YellowYawConfig
sys = CarSystem()
sys.calibrate()
sys.start_tickers()
IR_EN_PIN = 'D0'
# 打开红外补光灯/红外灯 EN
ir_en = Pin(IR_EN_PIN, Pin.OUT, value=False)
ir_en.high()
guide = GuideController(sys, sys.odo)
act = ActionController(sys)


time.sleep_ms(500)

act.go_straight(525,-2000)
sys.odo.reset()
#while True:
#sys.motor.set_velocity(0,0,-1000)
count=0
#ID=[1,2,4]
Y=[-875,-710,-980]

while True:
    sys.vision.send_switch_command(2)
    if count==1:
        ok = guide.goto_position(Y[count], -1540, hold_yaw=0.0,precise=False,min_creep_speed=2500)
    elif count==2:
        ok = guide.goto_position(Y[count], -1650, hold_yaw=0.0,precise=False,min_creep_speed=2500)
#                               ff_only_ms=1000)
    if count!=0:
        sys.vision.send_switch_command(-1)
        if count>=2:
            sys.vision.send_red_sandbag_enabled(False)
            
        if count==2:
            if not act.search_until_visible(count,-1000):
                #act.go_straight(600,-1000)
                if not act.search_until_visible(count,1000):
                    count += 1
                    sys.vision.send_switch_command(-1)
                    continue
        elif count==1:
            if not act.search_until_visible(count,1000):
                #act.go_straight(600,-1000)
                if not act.search_until_visible(count,-1000):
                    count += 1
                    sys.vision.send_switch_command(-1)
                    continue
    else:
        ok = guide.goto_position(Y[count], -1500, hold_yaw=0.0,precise=False,min_creep_speed=2500)
        act.wait(500)
        act.search_until_visible(count,1000)
    # search_until_visible 只有在目标连续稳定后才返回。
    if count==0:
        ID=2
    else:
        ID = sys.vision.get_class_id()
    sys.vision.send_switch_command(ID)

    
    straight_t = 0#低：3800 中：
    if sys.vision.get_distance_hint() < 0.72:
        if count==1:
            straight_t = (
                0.72 - sys.vision.get_distance_hint()
            ) * 2800
        elif count==0:
            if sys.vision.get_distance_hint() < 0.65 and sys.vision.get_distance_hint()!=0:
                straight_t = (
                    0.65 - sys.vision.get_distance_hint()
                ) * 5000
            else:
                straight_t = 0
            
        else:
            straight_t = (
                0.72 - sys.vision.get_distance_hint()
            ) * 4500

    if straight_t!=0:
        act.go_straight(straight_t, -1600)

    #======================这一段预赛注释掉即可====================
    if ID != 2:
        
        act.lateral_to_ready(ID)
        # 原来的可选分支保持注释：
        # if ID == 0 or ID == 1:
        #     while sys.odo.x > -1700:
        #         sys.motor.set_velocity(-1500, 0, 0)
        #     act.go_straight(1500, 800)
        act.wait(300)
        act.yaw_correct(ID)
    

    # 转到指定位置后恢复全画面。
    # 网球 ID=2 不执行 yaw_correct，因此在靠近动作结束后直接恢复。
    if ID==2:
        act.yaw_c_self()
        #act.go_straight(600,-1500)
    act.wait(500)
    act.lateral_to_ready(ID)
    act.wait(800)
    push_ok = act.push_until_stop_signal(ID)

    # =====================================================
    # 恢复原有流程：每个物体推出黄线后，先后退一截。
    #
    # vx > 0 表示后退。第三个物体也必须先执行这段，
    # 让已经越过画面下沿的黄线重新回到摄像头视野，
    # 然后再进入黄线航向模式校正 IMU。
    # =====================================================
    if ID != 2:
        act.go_straight(600, 1500)
        act.wait(500)
        act.yaw_c_self()

    act.wait(800)
    count+=1
    sys.vision.send_switch_command(-1) 
    if count==3:
        act.wait(500)
        act.return_to_dock(guide)
        
        timeout_ms=3800#改
        time0801=time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), time0801) < timeout_ms:
            #yaw = self.imu.get_yaw()
            sys.motor.set_velocity(0, 0, 1200)
            time.sleep_ms(1)
        '''act.yaw_c_self()        
        ok = guide.goto_position(-200, -200, hold_yaw=0.0,precise=False,min_creep_speed=2000)
        act.go_straight(700,1500)'''
        while True:
            sys.motor.set_velocity(0, 0, 0)
            

