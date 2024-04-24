import Sofa
import Sofa.Core

import tkinter as tkinter
import threading
import numpy as np
from math import pi


class App(threading.Thread):

    def __init__(self, q0):
        threading.Thread.__init__(self)
        self.daemon = True
        self.start()

        self.q0Init = q0

    def reset(self):
        for i in range(0, len(self.q0Init)):
          self.q[i].set(self.q0Init[i])

    def callback(self):
        self.root.quit()

    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        tkinter.Label(self.root, text="Robot Controller Interface").grid(row=0, columnspan=7)

        self.q = []
        for i in range(0, len(self.q0Init)):
          self.q.append(tkinter.DoubleVar())

        for i in range(0, len(self.q)):
          tkinter.Scale(self.root, variable=self.q[i], resolution=0.001, length=400, from_=-pi, to=pi, orient=tkinter.VERTICAL).grid(row=1, column=i)

        self.root.mainloop()


class RobotGUI(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.robot = kwargs["robot"]
        self.q0 = kwargs["q0"]
        self.app = App(self.q0)

        assert len(self.robot.q0) == len(self.q0)

        return

    def reset(self):
        self.app.reset()

    def onAnimateBeginEvent(self, event):

        if self.robot is None:
            return

        q = []
        for i in range(0, len(self.app.q)):
          q.append(self.app.q[i].get())

        self.robot.q0 = q

        return


# Test/example scene
def createScene(rootNode):

    from header import addHeader

    addHeader(rootNode)
    rootNode.addObject(RobotGUI(robot=None))

    return
