from lxml import etree

#index number of / joy.buttons:
#0 - A
#1 - B
#2 - X
#3 - Y
#4 - LB
#5 - RB
#6 - back
#7 - start
#8 - power
#9 - Button stick left
#10 - Button stick right

#index number of / joy.axis:
#0 - Left / Right Axis stick left
#1 - Up / Down Axis stick left
#2 - LT
#3 - Left / Right Axis stick right
#4 - Up / Down Axis stick right
#5 - RT
#6 - cross key left / right
#7 - cross key up / down

class ControlScheme:
    def __init__(self):
        self.axesTarget = []
        self.buttonsTarget = []
        self.targetControls = {}
        self.XMLfileNames = ["ControlScheme.xml"]
        self.index = 0

    def parseXML(self):
        for fileName in self.XMLfileNames:
            tree = etree.parse(fileName)
            root = tree.getroot()

            axes = [None]*8
            buttons = [None]*11

            for axis in root.findall("axis"):
                axes[int(axis.get("index"))] = axis.get("target")

            for button in root.findall("button"):
                buttons[int(button.get("index"))] = button.get("target")

            self.axesTarget.append(axes)
            self.buttonsTarget.append(buttons)

    def interpretJoyMsg(self, axes_values, buttons_values):
        for i in range(len(axes_values)):
            if(not self.axesTarget[self.index][i] == None):
                self.targetControls[self.axesTarget[self.index][i]] = axes_values[i]

        for i in range(len(buttons_values)):
            if(not self.buttonsTarget[self.index][i] == None):
                self.targetControls[self.buttonsTarget[self.index][i]] = buttons_values[i]
        print(self.targetControls)
        print("\n\n\n")

    def setIndex(self, n):
        self.index = n