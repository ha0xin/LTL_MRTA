"""
全局计时器模块
提供全局计时功能，避免层层传递start_time参数
"""

from datetime import datetime
from termcolor import cprint
from typing import Optional, Dict
import threading


class GlobalTimer:
    """全局计时器类"""

    def __init__(self):
        self._start_time: Optional[datetime] = None
        self._checkpoints: Dict[str, datetime] = {}
        self._lock = threading.Lock()  # 线程安全

    def start(self, message: str = "程序开始"):
        """开始计时"""
        with self._lock:
            self._start_time = datetime.now()
            self._checkpoints.clear()
            cprint(f"🚀 {message}", "green", attrs=["bold"])

    def checkpoint(self, task_name: str, show_duration: bool = False):
        """设置检查点并打印从开始到现在的时间"""
        if self._start_time is None:
            cprint("❌ 计时器尚未启动，请先调用 start()", "red")
            return

        with self._lock:
            current_time = datetime.now()
            elapsed = (current_time - self._start_time).total_seconds()

            # 计算从上一个检查点到现在的持续时间
            duration = None
            if show_duration and self._checkpoints:
                last_checkpoint_time = max(self._checkpoints.values())
                duration = (current_time - last_checkpoint_time).total_seconds()

            # 允许同名task_name重复，使用自增后缀区分
            base_name = task_name
            idx = 1
            while task_name in self._checkpoints:
                task_name = f"{base_name}_{idx}"
                idx += 1
            self._checkpoints[task_name] = current_time

            if duration is not None:
                cprint(f"⏰ {task_name}: {elapsed:.3f}s (持续时间: {duration:.3f}s)", "yellow", attrs=["bold"])
            else:
                cprint(f"⏰ {task_name}: {elapsed:.3f}s", "yellow", attrs=["bold"])

    def get_elapsed_time(self) -> float:
        """获取从开始到现在的总时间（秒）"""
        if self._start_time is None:
            return 0.0
        return (datetime.now() - self._start_time).total_seconds()

    def get_elapsed_datetime(self) -> Optional[datetime]:
        """获取已经过的时间作为datetime对象"""
        if self._start_time is None:
            return None
        return datetime.now() - self._start_time

    def reset(self):
        """重置计时器"""
        with self._lock:
            self._start_time = None
            self._checkpoints.clear()

    def summary(self):
        """打印所有检查点的摘要"""
        if self._start_time is None:
            cprint("❌ 计时器尚未启动", "red")
            return

        cprint("\n📊 计时摘要:", "cyan", attrs=["bold"])
        cprint("-" * 50, "cyan")

        for task_name, checkpoint_time in self._checkpoints.items():
            elapsed = (checkpoint_time - self._start_time).total_seconds()
            cprint(f"  {task_name}: {elapsed:.3f}s", "white")

        total_time = self.get_elapsed_time()
        cprint(f"  总计时间: {total_time:.3f}s", "cyan", attrs=["bold"])
        cprint("-" * 50, "cyan")


# 创建全局计时器实例
timer = GlobalTimer()


# 便捷函数
def start_timer(message: str = "程序开始"):
    """开始全局计时"""
    timer.start(message)


def log_time(task_name: str, show_duration: bool = False):
    """记录时间检查点"""
    timer.checkpoint(task_name, show_duration)


def get_elapsed_time() -> float:
    """获取已经过的时间（秒）"""
    return timer.get_elapsed_time()


def get_elapsed_datetime() -> Optional[datetime]:
    """获取已经过的时间作为datetime对象"""
    return timer.get_elapsed_datetime()


def reset_timer():
    """重置计时器"""
    timer.reset()


def show_summary():
    """显示计时摘要"""
    timer.summary()


# 向后兼容的函数
def print_time(task_str: str, time_str: float, duration: Optional[float] = None):
    """向后兼容的print_time函数"""
    if duration is not None:
        cprint(f"⏰ {task_str}: {time_str:.3f}s (Duration: {duration:.3f}s)", "yellow", attrs=["bold"])
    else:
        cprint(f"⏰ {task_str}: {time_str:.3f}s", "yellow", attrs=["bold"])
