# config.py
# 集中存放旧版已经跑通的参数。后续调试优先改这里，不要到处改逻辑代码。

class PinConfig:
    LED = 'C4'
    SWITCH2 = 'D9'

    # 旧版 motor_ctrl.py 已验证的编码器引脚
    ENCODER_A = ('C2', 'C3')
    ENCODER_B = ('D15', 'D16')
    ENCODER_C = ('D13', 'D14')


class MotorConfig:
    # 旧版已验证的电机 PWM 频率与 duty 限幅
    PWM_FREQ = 13000
    MAX_DUTY = 4000

    # 旧版编码器速度标定系数
    RATE_A = 9.63
    RATE_B = 9.20
    RATE_C = 9.496

    # 三轮运动学旋转权重，来自旧版 move() 的 L = 1 / 3
    ROTATE_L = 1.0 / 3.0


class PIDConfig:
    # 旧版 A/B/C 三路增量式 PID 参数
    A_KP = 0.27
    A_KI = 0.30
    A_KD = 0.00

    B_KP = 0.50
    B_KI = 0.15
    B_KD = 0.10

    C_KP = 0.50
    C_KI = 0.15
    C_KD = 0.15


class VisionConfig:
    UART_ID = 7
    UART_BAUD = 115200
    CENTER_OFFSET_X = 0.015

    # 摄像头临时丢失/跳变保护。
    # 摄像头约 6 FPS 时，650 ms 大约对应 3~4 帧。
    TARGET_TIMEOUT_MS = 900
    N_HOLD_MS = 650

    # 主板端再做一层候选确认，避免单帧错误直接进入运动控制。
    RX_CONFIRM_FRAMES = 2
    RX_SWITCH_CONFIRM_FRAMES = 3
    SEARCH_STABLE_FRAMES = 3

    # 相邻有效目标允许的最大跳变。
    RX_MATCH_CX = 0.18
    RX_MATCH_BOTTOM_Y = 0.18
    RX_MIN_SIZE_RATIO = 0.50

    # 坐标指数平滑系数；越大越跟手，越小越平稳。
    RX_SMOOTH_ALPHA = 0.55

    # 主板 -> 摄像头新增但不要求摄像头回 ACK 的命令：
    # R,1：只接受画面下部目标；R,0：恢复全画面。
    LOWER_ROI_ON_COMMAND = 'R,1\n'
    LOWER_ROI_OFF_COMMAND = 'R,0\n'
    ROI_COMMAND_REPEAT = 2
    ROI_COMMAND_GAP_MS = 20


    # 黄线航向标定通信。
    # 主板 -> 摄像头：
    #     IMU,1：进入黄线航向测量模式
    #     IMU,0：退出并恢复普通识别
    YAW_LINE_ON_COMMAND = 'IMU,1\n'
    YAW_LINE_OFF_COMMAND = 'IMU,0\n'

    # 主板端对摄像头 Y 数据再做一层过滤。
    YAW_LINE_TIMEOUT_MS = 900
    YAW_LINE_MIN_QUALITY = 350
    YAW_LINE_HISTORY_SIZE = 5
    YAW_LINE_STABLE_SAMPLES = 3
    YAW_LINE_STABLE_SPREAD_DEG = 2.5

    CLASS_NAMES = {
        0: 'sandbag.red',
        1: 'sandbag.blue',
        2: 'tennis',
        3: 'teddy.white',
        4: 'teddy.brown',
    }


class YellowYawConfig:
    # =====================================================
    # 第一次实车必须标定的三个参数
    # =====================================================

    # 车身机械上完全对正黄线时，摄像头测得的黄线角度。
    # 先保持 0.0，按照说明实测后填写。
    CAMERA_LINE_ZERO_DEG = 1.57

    # 图像黄线角度变化与 IMU yaw 变化的方向关系。
    #
    # 初值使用 +1.0，但必须通过正负 10° 实验确认：
    # 若 IMU yaw 增大时，摄像头黄线角度也增大，保持 +1；
    # 若一个增大、另一个减小，改为 -1。
    LINE_TO_YAW_SIGN = -2.0

    # 图像角度变化到车身真实 yaw 变化的比例。
    # 例如车实际转 10°，图像只变化 8°，填写 10/8=1.25。
    LINE_TO_YAW_SCALE = 1.0

    # =====================================================
    # 融合保护
    # =====================================================

    # 只融合视觉误差的 70%，第一次调试更安全。
    CORRECTION_GAIN = 0.50

    # 单次最多修正 12°，防止误检造成大跳变。
    MAX_CORRECTION_STEP_DEG = 8.0

    # 视觉绝对航向与当前 IMU 相差超过该值时拒绝。
    HARD_REJECT_ERROR_DEG = 40.0

    # 小于该值不修改，避免噪声来回抖动。
    MIN_CORRECTION_DEG = 0.30

    # 停车后等待车身和 IMU 稳定。
    VEHICLE_SETTLE_MS = 300

    # 最长等待稳定黄线数据的时间。
    ACQUIRE_TIMEOUT_MS = 3500

    # 退出摄像头黄线模式后等待普通识别恢复。
    MODE_EXIT_SETTLE_MS = 100

    # 由当前 actions.py 的 yaw_correct() 实际停止方向得到：
    #
    # 类别0/1：推物时约 -90°
    # 类别2：推物时约 0°
    # 类别3/4：推物时约 +90°
    EXPECTED_YAW_BY_CLASS = {
        0: -90.0,
        1: -90.0,
        2: 0.0,
        3: 90.0,
        4: 90.0,
    }


class IMUConfig:
    ACC_LSB_TO_G = 1.0 / 4096.0
    GYRO_LSB_TO_DEG = 1.0 / 16.4
    SCALE_FACTOR_Z = 1.0260
    CALIBRATE_SAMPLES = 200
    CALIBRATE_DELAY_MS = 5
    TWO_KP = 1.0
    TWO_KI = 0.0


class WirelessConfig:
    BAUD = 115200
    SEND_FRAME_LEN = 13
    RX_PAYLOAD_LEN = 9

    # AA55 速度帧（send_velocity 发出）去掉帧头 0xAA 0x55 后剩余字节数：
    # class_id(1) + ctrl(1) + weight_mode(1) + A(2) + B(2) + C(2) + checksum(1) + tail(1)
    VELOCITY_RX_PAYLOAD_LEN = 11


class TickerConfig:
    WIRELESS_ID = 0
    CONTROL_ID = 1
    VISION_ID = 2
    IMU_ID = 3

    WIRELESS_PERIOD_MS = 15
    CONTROL_PERIOD_MS = 10
    VISION_PERIOD_MS = 20
    IMU_PERIOD_MS = 5


class MotionConfig:
    # 旧版 main.py 中实际使用的速度量级
    SPEED_START_FORWARD = -2500
    SPEED_START_LATERAL = 800
    SPEED_SEARCH_SLOW = 800
    SPEED_SEARCH_FAST = 1200
    SPEED_H_CORRECT = 800
    SPEED_APPROACH = -1200
    SPEED_APPROACH_SLOW = -800
    SPEED_PUSH = -2000
    SPEED_BACK = 2000

    TURN_VY = 1500
    TURN_W = 800
    TURN_AROUND_VX = 300
    TURN_AROUND_VY = 500
    TURN_AROUND_W = -5000


class TaskConfig:

    # 完成第几个物体后执行黄线 IMU 校正。
    #
    # main.py 中 count 从 0 开始：
    #     推第1个时 count=0
    #     推第2个时 count=1
    #     推第3个时 count=2
    #
    # 推完以后 completed_count = count + 1，
    # 因此这里写 3，表示“第三个物体已经推出界线后”。
    ENABLE_YELLOW_IMU_CORRECTION = True
    IMU_CORRECT_AFTER_COMPLETED_COUNT = 3

    # 旧版任务顺序：ID=[1,0,0,3,4]
    TASK_SEQUENCE = [1, 0, 0, 3, 4]

    # 旧版 dist_min=[0.68,0.68,0.68,0.62,0.62]
    # 注意原代码按 id 索引，因此这里保留同样写法。
    DIST_MIN_BY_ID = {
        0: 0.68,
        1: 0.68,
        2: 0.68,
        3: 0.62,
        4: 0.62,
    }

    # 旧版 make_time=[15000,15000,8000,8000,3000]，按任务序号 count 索引。
    MAKE_TIME_BY_INDEX = [15000,15000,8000,8000,3000]

    # 旧版 ccnt=1：完成第 1 个任务后进入绿灯/停车流程。
    # 如果你要跑完整 5 个目标，把它改成 None。
    STOP_AFTER_COUNT = 1

    # 旧版横向纠偏参数表。原代码只给了 0/1/2；3/4 这里默认沿用 0 的保守参数，避免后续跑完整序列时报 KeyError。
    H_CORRECT_CFG = {
        0: {'SEARCH_TH': 0.030, 'PREDICT_K': 0.60, 'ERR_OVER': 0.04},
        1: {'SEARCH_TH': 0.030, 'PREDICT_K': 0.56, 'ERR_OVER': 0.06},
        2: {'SEARCH_TH': 0.010, 'PREDICT_K': 0.50, 'ERR_OVER': 0.03},
        3: {'SEARCH_TH': 0.030, 'PREDICT_K': 0.60, 'ERR_OVER': 0.04},
        4: {'SEARCH_TH': 0.030, 'PREDICT_K': 0.60, 'ERR_OVER': 0.04},
    }


class OdometryConfig:
    # 编码器测速单位 -> 实际线速度 mm/s 的换算系数。
    SPEED_TO_MMPS = 0.2121  # TODO: 必须实测标定，否则位置估计的比例是错的
    #0.2105，0.2138
    # 里程计积分的 dt 保护，避免 ticker 卡顿时单次积分距离过大。
    DT_MAX = 0.05


class GuideConfig:
    # 位置环比例增益：车体坐标误差(mm) -> 速度指令，与 set_velocity 的 vx/vy 同量级，需要实测调参。
    POS_KP = 3.7

    # 航向保持环比例增益：角度误差(deg) -> w 指令。
    YAW_KP = 40.0

    MAX_SPEED = 2500#改过
    MAX_YAW_RATE = 1000

    # timed_move（go_straight/go_lateral/推物前进后退等）里航向保持用的死区，
    # 单位度。误差小于这个值时不产生 w 修正，避免 IMU 噪声导致轮子持续微调。
    YAW_HOLD_DEADZONE_DEG = 1.5

    # 到点判定容差，单位 mm。precise=True（默认，精确模式）时用这个。
    POS_TOLERANCE_MM = 30.0

    # precise=False（快速模式）时的默认到点容差，单位 mm。
    # 也可以在调用 goto_position 时用 tolerance_mm 参数按任务单独指定，
    # 不必局限于这两档默认值。
    FAST_POS_TOLERANCE_MM = 100.0

    # 距离小于该半径时按比例减速，避免冲过目标点。仅精确模式使用。
    SLOWDOWN_RADIUS_MM = 300.0
    MIN_SPEED_SCALE = 0.25

    # 减速半径相对到点容差的倍数下限：不管调用者传入多大的 tolerance_mm，
    # 减速区半径至少是它的这么多倍，避免车辆还没进入减速阶段、
    # 仍按全速跑时就因为容差设得大直接被判定"到点"而急停冲出去。
    SLOWDOWN_RADIUS_RATIO = 3.0

    # 死区补偿：vx/vy 指令非零时的最小幅值，单位与 set_velocity 的 vx/vy 同量级。
    #怀疑是机械阻力问题（轮子/轴承卡顿、
    # 三轮承重不均等），而不是纯粹的死区。
    # 仅精确模式使用，快速模式改用方向锁定，不再逐轴做死区补偿。
    MIN_CREEP_SPEED = 200.0

    # 快速模式（precise=False）：目标方向角变化超过该阈值才重新锁定方向，
    # 用来吃掉接近目标点时因位置噪声导致的方向抖动（Z字形）。
    FAST_LOCK_ANGLE_DEG = 20.0

    # 快速模式：锁定方向后使用的固定速度幅值，先用 MAX_SPEED 全速冲向目标。
    FAST_LOCK_SPEED = 2500.0

    # goto_position(ff_only_ms=...) 用：导航开始的前 ff_only_ms 毫秒只让车2用前馈
    # （不看红外纠偏），之后切到前馈+红外融合。对应 car2R FollowConfig.WEIGHT_PRESETS
    # 的 key，具体权重值在 car2R 侧定义。
    FF_ONLY_WEIGHT_MODE = 2      # 只开前馈（vision权重0）
    COMBINED_WEIGHT_MODE = 0     # 前馈+红外融合（normal默认组合）
    FF_ONLY_DURATION_MS = 1000

    DEFAULT_TIMEOUT_MS = 8000

class CoopConfig:
    # 无线 weight_mode 约定：
    # 0：普通阶段，车2使用常规自转补偿参数。
    # 4：推物完成后，主车仅发送 w 原地自转回场阶段，
    #    车2切换到第二套自转补偿参数。
    NORMAL_WEIGHT_MODE = 0
    RETURN_SPIN_WEIGHT_MODE = 4

    # 主车切换 weight_mode 后，等待若干无线周期再开始/结束动作，
    # 确保车2已经收到新模式。无线周期为 20 ms，60 ms 约为 3 帧。
    WEIGHT_MODE_SETTLE_MS = 60



