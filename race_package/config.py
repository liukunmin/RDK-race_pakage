# config.py
# 所有可调参数集中在此
# ============================================================

class Config:
    # ==================== 一、坐标点 ====================
    
    PRE_QR_X = 1.5            # 二维码前置点 X坐标
    PRE_QR_Y = -1.5           # 二维码前置点 Y坐标
    PRE_QR_TOLERANCE = 1.8    # 二维码前置点到达容忍（米）

    QR_GOAL_X = 2.0          # 二维码点 X坐标
    QR_GOAL_Y = -1.0         # 二维码点 Y坐标
    QR_GOAL_YAW = 1.0        # 二维码点目标朝向（弧度），0.785=45° 
    ARRIVAL_TOLERANCE = 0.4  # 二维码点到达容忍（米）

    # ==================== 二、黄色通道路径点 ====================
    PATH_COUNTER_CLOCKWISE = [
        (0.0, 0.3),    # 0: 黄色通道入口

        (1.8, 0.5),    # 1: 右下
        (1.7, 1.8),    # 2: 右上
        (-1.7, 1.8),   # 3: 左上
        (-1.8, 0.5),   # 4: 左下

        (0.0, 0.4),    # 5: 过渡点
        (-2.1, -2.3),  # 6: 返回P点
    ]
    
    PATH_CLOCKWISE = [
        (0.0, 0.4),    # 0: 黄色通道入口

        (-1.8, 0.8),   # 1: 左下
        (-1.7, 1.7),   # 2: 左上
        (1.7, 1.8),    # 3: 右上
        (1.6, 0.5),    # 4: 右下

        (0.0, 0.3),    # 5: 过渡点
        (-2.1, -2.3),  # 6: 返回P点
    ]
    
    # ==================== 三、到达容忍范围 ====================
    WAYPOINT_TOLERANCES = {
        0: 1.2,   # 入口
        1: 0.8,   # 右下/左下
        2: 0.8,   # 右上/左上
        3: 0.8,   # 立牌识别点（减小容忍让小车更靠近立牌再拍照）
        4: 0.8,   # 左下/右下
        5: 1.0,   # 过渡点
        6: 0.5,   # 返回P点（精确）
    }
        
    
    # ==================== 五、锥桶检测 ====================
    SAFE_STOP_DIST = 0.2        # 强制停止距离（米
    
    # ==================== 七、静止检测 ====================
    STUCK_TIMEOUT = 1.0         # 静止超时（秒）
    GOAL_TIMEOUT = 8.0          # 导航超时（秒）
    
    # ==================== 八、智能移动速度 ====================
    FORWARD_SPEED = 0.2                # 智能前进速度（m/s）
    BACKWARD_SPEED = -0.3              # 智能后退速度（m/s）
    MAX_MOVE_TIME = 1.0                # 最大移动时间（秒）
    SAFE_DIST = 0.2                    # 扇区安全距离阈值（米）
    MIN_MOVE_SPACE = 0.2               # 最小移动空间阈值（米），不足则切换方向
    SPACE_SHRINK_RATIO = 0.60          # 空间缩小比率阈值，低于此值停止
    MOVE_STABILIZE_TIME = 0.5          # 启动后稳定期（秒）
    CMD_PUBLISH_PERIOD = 0.02          # cmd_vel 发布周期（秒）
    SPACE_CHECK_PERIOD = 0.1           # 空间检查周期（秒）
    DELAYED_GOAL_TIME = 0.3            # 延迟重发目标时间（秒）
    MIN_VALID_DIST = 0.1               # 最小有效距离过滤（米）

    # ==================== 九、圆弧调整参数 ====================
    ANGLE_THRESHOLD = 60.0        # 夹角阈值（度）— 每个动作完成后检查，达标立即退出
    ROTATION_MAX_CYCLES = 3       # 圆弧调整最大循环次数（安全上限，正常情况下1-2个动作即可达标）
    ROTATION_BACK_SPEED = -0.5    # 后退速度（负值表示后退）
    ROTATION_BACK_STEERING = 1.0  # 后退时角速度（正值，因 linear<0 时正角速度→右转向）
    ROTATION_BACK_DURATION = 0.6  # 后退持续时间（秒）
    ROTATION_FWD_SPEED = 0.6      # 前进速度
    ROTATION_FWD_STEERING = 1.0   # 前进时转向角（与后退方向相反）
    ROTATION_FWD_DURATION = 0.7   # 前进持续时间（秒）
    ROTATION_CONE_THRESHOLD = 0.3 # 圆弧调整中锥桶检测阈值（米）
    
    # 圆弧调整节拍间隔（秒），实车电机响应延迟补偿
    ROTATION_PAUSE_DURATION = 0.1 # 后退与前进之间的暂停间隔
    
    # 圆弧调整中锥桶检测范围（度），前方 ±30°
    ROTATION_CONE_SCAN_LOW = -45
    ROTATION_CONE_SCAN_HIGH = 45

    # ==================== 十一、大模型识别配置 ====================
    # 图像传输话题
    IMAGE_TOPIC = '/sign_image'           # 小车发布图像 → 笔记本订阅
    RESULT_TOPIC = '/sign_result'         # 笔记本发布结果 → 小车订阅
    # 立牌触发距离
    SIGN_TRIGGER_DIST = 0.4               # 到达立牌多少米内触发拍照
    # 二维码检测配置
    QR_DETECT_INTERVAL = 3               # 每N帧检测一次（降低CPU占用）
    QR_LOG_INTERVAL = 30                 # 每N帧打印一次收图日志
    QR_ZOOM_FACTORS = [2.0, 3.0]         # 依次尝试的放大倍数
    QR_MIN_SIZE = 10                     # 透视矫正最小QR尺寸（像素）
    
     # ==================== 十二、智能移动扫描扇区 ====================
    # 智能后退三方向扇区（角度范围，单位：度）
    # 0° = 正后方，负值 = 左，正值 = 右
    BACK_SECTOR_STRAIGHT_LOW = -20
    BACK_SECTOR_STRAIGHT_HIGH = 20
    BACK_SECTOR_RIGHT_LOW = 50
    BACK_SECTOR_RIGHT_HIGH = 80
    BACK_SECTOR_LEFT_LOW = -80
    BACK_SECTOR_LEFT_HIGH = -50
    # 后退优先级（可调整顺序）
    BACKWARD_PRIORITY = ["left", "straight", "right"]

    # 智能前进扫描范围（对称，无优先级）
    FRONT_SCAN_LOW = -60
    FRONT_SCAN_HIGH = 60
    FRONT_CENTER_LOW = -15            # 直前进中心扇区下限（度）
    FRONT_CENTER_HIGH = 15            # 直前进中心扇区上限（度）
    SCAN_ANGLE_STEP = 5               # 扫描角度步长（度）

    # 智能移动转向角
    BACKWARD_STEERING_RIGHT = -0.4    # 右后退转向角
    BACKWARD_STEERING_LEFT = 0.4      # 左后退转向角
    FORWARD_STEERING_LEFT = 0.4       # 左前进转向角
    FORWARD_STEERING_RIGHT = -0.4     # 右前进转向角