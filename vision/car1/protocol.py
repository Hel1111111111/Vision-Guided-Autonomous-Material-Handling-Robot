# -*- coding: utf-8 -*-
from machine import UART

import config


class MainboardProtocol:
    """
    保留原有类别通信和下部 ROI 命令，并新增：

        V,0     关闭全部视觉识别
        V,1     恢复全部视觉识别

        RED,0   忽略红沙包类别 0
        RED,1   恢复红沙包类别 0

    主板 -> 摄像头：
        C,-1 / C,0~4
        -1 / 0~4

        R,1     只接受画面下部目标
        R,0     恢复全画面目标

        V,0 / V,1
        RED,0 / RED,1

        IMU,1   进入黄线航向测量模式
        IMU,0   退出黄线航向测量模式

    摄像头 -> 主板：
        N
        T,label,score,cx,cy,w,h,bottom_y
        STOP
        Y,angle_x100,quality,cy_x1000
        YN

    新命令均不返回 ACK；关闭普通识别后摄像头不发送 N/T/STOP。
    黄线航向模式只发送 Y/YN。
    """

    def __init__(self):
        self.uart = UART(
            config.UART_ID,
            config.UART_BAUD,
            timeout_char=config.UART_TIMEOUT_CHAR
        )

        self.buffer = b""

    def poll(self):
        commands = []

        try:
            if self.uart.any():
                data = self.uart.read()

                if data:
                    self.buffer += data

            if len(self.buffer) > 256:
                self.buffer = self.buffer[-128:]

            while b"\n" in self.buffer:
                raw_line, self.buffer = (
                    self.buffer.split(b"\n", 1)
                )

                try:
                    line = raw_line.decode().strip()
                except Exception as error:
                    print("UART DECODE ERR:", error)
                    continue

                if not line:
                    continue

                command = self._parse_command(line)

                if command is not None:
                    commands.append(command)

        except Exception as error:
            print("UART CMD ERR:", error)

        return commands

    def _parse_command(self, line):
        upper = line.upper()

        if upper == "IMU,1":
            return {
                "type": "yaw_line",
                "enabled": True
            }

        if upper == "IMU,0":
            return {
                "type": "yaw_line",
                "enabled": False
            }

        if upper == "V,0":
            return {
                "type": "recognition",
                "enabled": False
            }

        if upper == "V,1":
            return {
                "type": "recognition",
                "enabled": True
            }

        if upper == "RED,0":
            return {
                "type": "red_sandbag",
                "enabled": False
            }

        if upper == "RED,1":
            return {
                "type": "red_sandbag",
                "enabled": True
            }

        if upper == "R,1":
            return {
                "type": "roi",
                "lower_only": True
            }

        if upper == "R,0":
            return {
                "type": "roi",
                "lower_only": False
            }

        target_class = None

        try:
            if upper.startswith("C,"):
                target_class = int(
                    line.split(",", 1)[1]
                )
            else:
                target_class = int(line)
        except Exception:
            return None

        if (
            target_class == -1
            or 0 <= target_class < len(config.LABELS)
        ):
            return {
                "type": "target",
                "value": target_class
            }

        return None

    def send_no_target(self):
        self.uart.write("N\n")

    def send_target(self, target):
        message = (
            "T,{},{},{},{},{},{},{}\n"
        ).format(
            int(target["label"]),
            int(target["score"] * 1000),
            int(target["cx"] * 1000),
            int(target["cy"] * 1000),
            int(target["w"] * 1000),
            int(target["h"] * 1000),
            int(target["bottom_y"] * 1000)
        )

        self.uart.write(message)

    def send_stop(self):
        self.uart.write("STOP\n")

    def send_yaw_line(self, result):
        """
        Y,angle_x100,quality,cy_x1000
        """
        message = (
            "Y,{},{},{}\n"
        ).format(
            int(result["angle_deg"] * 100.0),
            int(result["quality"]),
            int(result["cy"] * 1000.0)
        )

        self.uart.write(message)

    def send_yaw_line_none(self):
        self.uart.write("YN\n")
