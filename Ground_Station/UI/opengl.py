from PyQt5.uic.properties import QtWidgets, QtGui, QtCore


class grahicOpenGL(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None, picFile=None):
        super().__init__(parent)
        self.img = None
        self.picFile = None
        if picFile: self.loadPicFile(picFile)

    def loadPicFile(self, picFile):
        self.picFile = picFile
        self.img = QtGui.QImage()
        self.img.load(picFile)

    def paintGL(self):
        if self.img:
            paint = QtGui.QPainter()
            paint.begin(self)
            paint.drawImage(QtCore.QPoint(0, 0), self.img)
            paint.end()
