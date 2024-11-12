import Sofa
import Sofa.Core

import tkinter as tkinter
import threading
import numpy as np
from math import pi


class App(threading.Thread):

    def __init__(self, qRest):
        threading.Thread.__init__(self)
        self.daemon = True
        self.start()

        print("============ qRest size : ", len(qRest))
        self.qRest = qRest

    def reset(self):
        for i in range(0, len(self.qRest)):
          self.q[i].set(self.qRest[i])

    def callback(self):
        self.root.quit()

    def updateQLabels(self, event, index):
        self.qLabels[index].config(text=str(event))

    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.q = []
        for i in range(0, len(self.qRest)):
          self.q.append(tkinter.DoubleVar())

        self.qLabels = []
        for i in range(0, len(self.q)):
          tkinter.Label(self.root, text='q[' + str(i) + ']').grid(row=i, column=0)
          tkinter.Scale(self.root, variable=self.q[i], resolution=0.001, length=300, showvalue=0, from_=-pi, to=pi, orient=tkinter.HORIZONTAL, command=lambda event, index=i: self.updateQLabels(event, index)).grid(row=i, column=1)
          label = tkinter.Label(self.root, text=str(self.q[i].get()))
          label.grid(row=i, column=2)
          self.qLabels.append(label)

        self.root.mainloop()


class RobotGUI(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.robotNode = kwargs["robot"]

        return

    def init(self):
        # XXX: find a way to use link path to q0 data instead
        urdfLoader = self.robotNode.getObject('URDFModelLoader')
        qRest = urdfLoader.getData('qRest').value

        self.app = App(qRest)


    def reset(self):
        self.app.reset()

    def onAnimateBeginEvent(self, event):
        if self.robotNode is None:
            return

        # XXX: find a way to use link path to q0 data instead
        urdfLoader = self.robotNode.getObject('URDFModelLoader')
        qRestData = urdfLoader.getData('qRest')
        with qRestData.writeableArray() as qRestDataValue:
          for i in range(0, len(qRestDataValue)):
            qRestDataValue[i] = self.app.q[i].get()

        return


# Test/example scene
def createScene(rootNode):

    from header import addHeader

    addHeader(rootNode)
    rootNode.addObject(RobotGUI(robot=None))

    return
