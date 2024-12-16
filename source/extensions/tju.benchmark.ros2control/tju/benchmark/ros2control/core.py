from ._base import *
from .ros2 import IsaacSimNode

class SimControl():
    @classmethod
    def instance(cls):
        if not hasattr(cls, "_instance"):
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        self.node = IsaacSimNode()

    def start(self):
        self.node.start()

    def stop(self):
        self.node.stop()

    def update(self):
        pass
