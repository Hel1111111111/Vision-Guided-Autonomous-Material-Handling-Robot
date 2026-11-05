# task_runner.py
# 任务层：把旧版 main.py 中的大 while/count/ID 逻辑拆成可读的步骤。

import time
import gc
from config import TaskConfig, MotionConfig
from actions import ActionController


class TaskRunner:
    def __init__(self, system):
        self.sys = system
        self.act = ActionController(system)
        self.tasks = TaskConfig.TASK_SEQUENCE
        self.count = 0

    def startup_position(self):
        # 旧版 count==0 前置动作：go_straight(1400), wait(300), go_h(800,-1)
        first_id = self.tasks[0]
        self.sys.vision.send_switch_command(first_id)
        self.act.go_straight(1400)
        self.act.wait(300)
        self.act.go_lateral(800, -1)

    def prepare_before_search(self, index, class_id):
        """复现旧版 count==0/1/2/3/4 进入找目标前的位姿调整。"""
        self.sys.vision.send_switch_command(class_id)

        if index == 0:
            self.sys.ctrl = 0
            # 原代码：mc.err_x=0 后向 -800 横移搜索
            self.sys.velocity_weight_mode = 1
            self.act.search_until_visible(-800)
            self.act.align_fixed_direction(-800, 0.05)
            self.sys.velocity_weight_mode = 0

        elif index == 1:
            self.act.yaw_correct(0)
            self.act.wait(300)
            self.act.go_straight(1000)
            self.act.wait(300)
            self.sys.ctrl = 0
            self.act.go_lateral(250, -1)
            self.sys.velocity_weight_mode = 1
            self.act.search_until_visible(-1200)
            # 旧版这里根据 err_x 正负选方向
            while abs(self.sys.vision.get_error_x()) > 0.05:
                if self.sys.vision.get_error_x() > 0:
                    self.sys.motor.set_velocity(0, -1200, 0)
                else:
                    self.sys.motor.set_velocity(0, 1200, 0)
                time.sleep_ms(1)
            self.sys.motor.stop()
            self.sys.velocity_weight_mode = 0

        elif index == 2:
            self.act.yaw_correct(1)
            self.act.wait(300)
            self.act.go_straight(700)
            self.act.wait(300)
            self.act.go_lateral(300, 1)
            self.sys.ctrl = 0
            self.sys.velocity_weight_mode = 1
            self.act.search_until_visible(1200)
            self.act.align_fixed_direction(1200, 0.06)
            self.sys.velocity_weight_mode = 0

        elif index == 3:
            self.act.yaw_correct(0)
            self.act.wait(300)
            self.act.go_straight(2500)
            self.act.wait(300)
            self.sys.ctrl = 0
            self.sys.velocity_weight_mode = 1
            self.act.search_until_visible(-1200)
            self.act.align_fixed_direction(-1200, 0.06)
            self.sys.velocity_weight_mode = 0

        elif index == 4:
            self.act.yaw_correct(0)
            self.act.wait(300)
            self.act.go_straight(1700)
            self.act.wait(300)
            self.sys.ctrl = 0
            self.sys.velocity_weight_mode = 1
            self.act.search_until_visible(1200)
            self.act.align_fixed_direction(1200, 0.06)
            self.sys.velocity_weight_mode = 0

    def handle_target(self, index, class_id):
        # 1. 横向预测纠偏
        self.act.horizontal_correct(class_id)
        self.act.wait(500)

        # 2. 纵向靠近
        target_dist = TaskConfig.DIST_MIN_BY_ID.get(class_id, 0.68)
        self.act.approach_until_distance(target_dist, MotionConfig.SPEED_APPROACH)
        self.act.wait(500)

        # 3. 转大弯到投放/推动方向
        self.act.turn_delivery_by_id(class_id)

        # 4. 如果转弯后横向误差仍大，先前压一小段再纠偏
        self.act.wait(300)
        if abs(self.sys.vision.get_error_x()) > 0.03:
            self.act.timed_move(250, 2000, 0, 0)
            self.act.horizontal_correct(class_id)
            self.act.wait(300)

        # 5. 继续前进至 bottom_y=0.85，最多 3 秒
        self.act.approach_until_distance(0.85, MotionConfig.SPEED_APPROACH_SLOW, timeout_ms=3000)

        # 6. 航向角恢复
        self.act.yaw_recover_after_delivery(class_id)
        self.act.wait(500)

        # 7. 一车校准完，二车校准：一车等待 mov
        self.sys.ctrl = 1
        self.act.wait(1000)
        while self.sys.wireless.mov == 1:
            self.sys.motor.set_velocity(0, 0, 0)
            time.sleep_ms(1)

        # 8. 推动至目标处，直到视觉 STOP 或超时
        self.act.push_until_stop_signal(index)

    def final_green_stop_sequence(self):
        # 旧版 count==ccnt 后的绿灯停车流程
        self.sys.ctrl = 0
        self.sys.vision.reset_target()
        self.sys.vision.send_switch_command(5)

        self.act.wait(300)
        self.act.yaw_correct(1)
        self.act.wait(300)
        self.act.go_straight(800)

        self.act.search_until_visible(1200)
        self.act.align_fixed_direction(1200, 0.06)
        self.act.approach_until_distance(0.58, MotionConfig.SPEED_APPROACH)

        self.sys.ctrl = 1
        while True:
            self.sys.motor.set_velocity(0, 0, 0)
            time.sleep_ms(10)

    def turnaround_for_next_task(self):
        self.sys.ctrl = 0
        self.act.timed_move(510, MotionConfig.TURN_AROUND_VX, MotionConfig.TURN_AROUND_VY, MotionConfig.TURN_AROUND_W)
        self.sys.ctrl = 1
        self.act.wait(2000)
        while self.sys.wireless.mov == 1:
            self.sys.motor.set_velocity(0, 0, 0)
            time.sleep_ms(1)
        self.sys.vision.reset_target()

    def run(self):
        self.startup_position()

        while self.count < len(self.tasks):
            class_id = self.tasks[self.count]
            print('TASK START index=', self.count, 'class_id=', class_id)

            self.prepare_before_search(self.count, class_id)
            self.handle_target(self.count, class_id)

            self.count += 1
            print('TASK DONE count=', self.count)

            if TaskConfig.STOP_AFTER_COUNT is not None and self.count == TaskConfig.STOP_AFTER_COUNT:
                self.final_green_stop_sequence()

            self.turnaround_for_next_task()

            if self.sys.should_stop_by_switch():
                print('Switch changed, stop program.')
                break

            gc.collect()

        self.sys.stop()
        print('All tasks finished')
