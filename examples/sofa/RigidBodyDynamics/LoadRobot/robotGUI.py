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

        self.qRest = qRest
        self.q = np.copy(qRest)

    def reset(self):
        self.q =  np.copy(self.qRest)

    def callback(self):
        self.root.quit()

    def updateQLabels(self, event, index):
       val_str = str(event).replace(',', '.')
       self.q[index] = float(val_str)
       self.qLabels[index].config(text=val_str)

    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.q_sliders = []
        for i in range(0, len(self.q)):
          self.q_sliders.append(tkinter.DoubleVar(value=self.q[i]))

        self.qLabels = []
        for i in range(0, len(self.q)):
          tkinter.Label(self.root, text='q[' + str(i) + ']').grid(row=i, column=0)
          scale = tkinter.Scale(self.root,
                        variable=self.q_sliders[i],
                        resolution=0.001,
                        length=300,
                        showvalue=0,
                        from_=-pi,
                        to=pi,
                        orient=tkinter.HORIZONTAL,
                        command=lambda event, index=i: self.updateQLabels(event, index))
          scale.grid(row=i, column=1)
          label = tkinter.Label(self.root, text=str(self.q[i]))
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
            qRestDataValue[i] = self.app.q[i]

        return


# Test/example scene
def createScene(rootNode):

    from header import addHeader

    addHeader(rootNode)
    rootNode.addObject(RobotGUI(robot=None))

    return
