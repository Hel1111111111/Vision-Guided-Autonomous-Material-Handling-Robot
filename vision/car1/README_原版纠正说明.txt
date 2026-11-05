原版色块二次纠正恢复版
======================

一、恢复了什么
--------------
恢复原程序的色块二次纠正：

1. 模型输出初始框。
2. 初始框 x 向左偏移 0.1。
3. 在模型框相同 y、高度的整幅图像横条中搜索颜色。
4. 选择面积最大的对应色块。
5. 最终框 x、w 使用色块结果。
6. 最终框 y、h 使用模型结果。
7. 最终框经过时域滤波后发送主板。

二、仍然删除的功能
------------------
本工程没有：

    绿色区域
    类别 5
    红砖识别
    B,0 / B,1

保留：

    五类物品
    黄线 STOP
    UART2 主板通信
    固定相机参数

三、复制文件
------------
复制到 OpenMV/OpenART 根目录：

    main.py
    config.py
    camera_profile.py
    camera_init.py
    vision_utils.py
    model_detector.py
    target_filter.py
    yellow_line.py
    protocol.py

模型：

    /sd/yolo3_iou_smartcar_final_with_post_processing.tflite

四、直接测试
------------
运行 competition_debug.py，不需要主板。

显示：

    黄色框：模型原始框
    蓝色框：整行颜色搜索 ROI
    绿色框：颜色搜索选中的最大色块
    红色框：最终纠正框

五、正式运行
------------
运行 main.py。

默认：

    START_MODE = MODE_OBJECT

所以立即识别。

若正式比赛等待主板到达物品区后再识别：

    START_MODE = MODE_IDLE

主板发送：

    C,-1\n
    C,0\n ... C,4\n
    M,0\n
    M,1\n

返回：

    N
    T,label,score,cx,cy,w,h,bottom_y
    STOP

六、关闭青色黄线 ROI
--------------------
config.py：

    DRAW_YELLOW_ROI = False

已经默认关闭。

七、关闭原版纠正
----------------
若要临时对比模型原始效果：

    ENABLE_ORIGINAL_COLOR_CORRECTION = False

八、重要提示
------------
原版算法会在同一高度的整幅画面中选择最大色块，因此其他同色物体、
反光或背景可能抢占最终框。这是原算法本身的行为，本工程按要求恢复，
没有改成局部 ROI。
