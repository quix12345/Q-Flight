import _thread
import numpy as np
from PyQt5.QtGui import QColor, QIcon
from UI.MainWindow import Ui_MainWindow
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSignal, Qt, QThread
from utils.communication import Quadrotor, Communication
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# 前景（坐标轴，网格）消除锯齿
pg.setConfigOptions(foreground=QColor(113, 148, 116), antialias=True)


class GraphUpdateThread(QThread):
    updateGraph = QtCore.pyqtSignal()

    def __init__(self):
        super(GraphUpdateThread, self).__init__()

    def run(self):
        self.updateGraph.emit()


class ComUpdateThread(QThread):
    updated = QtCore.pyqtSignal(list)

    def __init__(self, mainWindow):
        super(ComUpdateThread, self).__init__()
        self.mainWindow = mainWindow

    def run(self):
        if self.mainWindow.is_Com_open and not self.mainWindow.is_Com_puase:
            self.updated.emit(self.mainWindow.Engine.buffer_string)


class MainPageWindow(QWidget, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainPageWindow, self).__init__(parent)

        self.setupUi(self)
        self.quadrotorDisplay = gl.GLViewWidget(self.horizontalLayoutWidget)
        self.quadrotorDisplay.setObjectName("quadrotorDisplay")
        self.quadrotorDisplay.show()
        self.quadrotorDisplay.setCameraPosition(distance=90)
        self.horizontalLayout.addWidget(self.quadrotorDisplay)
        g = gl.GLGridItem()
        g.scale(4, 4, 1)
        g.translate(0, 0, -20)
        self.quadrotorDisplay.addItem(g)
        g = gl.GLGridItem()
        g.scale(4, 4, 1)

        g.rotate(90, 1, 0, 0)
        g.translate(0, -40, 20)
        self.quadrotorDisplay.addItem(g)
        g = gl.GLGridItem()
        g.scale(4, 4, 1)

        g.rotate(-90, 0, 1, 0)
        g.translate(-40, 0, 20)
        self.quadrotorDisplay.addItem(g)

        def psi(i, j, k, offset=(25, 25, 50)):
            x = i - offset[0]
            y = j - offset[1]
            z = k - offset[2]
            th = np.arctan2(z, (x ** 2 + y ** 2) ** 0.5)
            phi = np.arctan2(y, x)
            r = (x ** 2 + y ** 2 + z ** 2) ** 0.5
            a0 = 1
            # ps = (1./81.) * (2./np.pi)**0.5 * (1./a0)**(3/2) * (6 - r/a0) * (r/a0) * np.exp(-r/(3*a0)) * np.cos(th)
            ps = (1. / 81.) * 1. / (6. * np.pi) ** 0.5 * (1. / a0) ** (3 / 2) * (r / a0) ** 2 * np.exp(
                -r / (3 * a0)) * (3 * np.cos(th) ** 2 - 1)
            return ps

        data = np.abs(np.fromfunction(psi, (50, 50, 100)))
        verts, faces = pg.isosurface(data, data.max() / 4.)
        md = gl.MeshData(vertexes=verts, faces=faces)
        colors = np.ones((md.faceCount(), 4), dtype=float)
        colors[:, 3] = 0.2
        colors[:, 2] = np.linspace(0, 1, colors.shape[0])
        md.setFaceColors(colors)
        m2 = gl.GLMeshItem(meshdata=md, smooth=True, shader='balloon')
        m2.setGLOptions('additive')
        m2.translate(-25, -25, -50)
        self.quadrotor_mesh = m2
        self.quadrotorDisplay.addItem(self.quadrotor_mesh)

        self.flightDataMonitorWidget = pg.PlotWidget(self.Total_tab_2)
        self.flightDataMonitorWidget.setGeometry(QtCore.QRect(0, 0, 651, 241))
        self.flightDataMonitorWidget.setObjectName("flightDataMonitorWidget")
        self.PID_Line_Edit = [self.PID_P_1, self.PID_I_1, self.PID_D_1,
                              self.PID_P_2, self.PID_I_2, self.PID_D_2,
                              self.PID_P_3, self.PID_I_3, self.PID_D_3,
                              self.PID_P_4, self.PID_I_4, self.PID_D_4,
                              self.PID_P_5, self.PID_I_5, self.PID_D_5,
                              self.PID_P_6, self.PID_I_6, self.PID_D_6,
                              self.PID_IL_1, self.PID_IL_2, self.PID_IL_3,
                              self.PID_IL_4, self.PID_IL_5, self.PID_IL_6]
        self.PID_Output_Group = [self.PID_Output_P_1, self.PID_Output_P_2, self.PID_Output_P_3,
                                 self.PID_Output_P_4, self.PID_Output_I_4, self.PID_Output_D_4,
                                 self.PID_Output_P_5, self.PID_Output_I_5, self.PID_Output_D_5,
                                 self.PID_Output_P_6, self.PID_Output_I_6, self.PID_Output_D_6,
                                 self.PID_Output_Final_1, self.PID_Output_Final_2, self.PID_Output_Final_3
            , self.PID_Output_Final_4, self.PID_Output_Final_5, self.PID_Output_Final_6]
        self.PID_CtrlLabelName_Group = [["Pitch外环", "Roll外环", "Yaw外环", "Pitch内环", "Roll内环", "Yaw内环"],
                                        ["速度X环", "速度Y环", "速度Z环", "位置X环", "位置Y环", "位置Z环"]]
        self.PID_CtrlLabel_Group = [self.PID_label_1, self.PID_label_2, self.PID_label_3, self.PID_label_4,
                                    self.PID_label_5, self.PID_label_6]
        self.PID_OutLabelName_Group = [["Pitch外环", "Roll外环", "Yaw外环", "Pitch内环", "Roll内环", "Yaw内环"],
                                       ["速度X环(°)", "速度Y环(°)", "速度Z环（°）", "位置X环(cm/s)", "位置Y环(cm/s)", "位置Z环(cm/s)"]]
        self.PID_OutLabel_Group = [self.PID_Output_label_1, self.PID_Output_label_2, self.PID_Output_label_3,
                                   self.PID_Output_label_4, self.PID_Output_label_5, self.PID_Output_label_6]

        self.PID_CtrlMode = 0
        self.PID_ReportMode = 0
        self.Buzzer_Status_Num = 0
        self.initUI()
        self.is_Com_open = False
        self.is_GrondStation_control = False
        self.is_Com_puase = False
        self.is_Com_continous = False
        self.is_Com_hex = True
        self.is_Com_updating = True
        self.is_Com_been_on = False
        self.is_Curve_Display = False
        self.comUpdateThread = ComUpdateThread(self)
        self.comUpdateThread.updated.connect(self.Update_TextBrowser)

        self.graphUpdateThread = GraphUpdateThread()
        self.graphUpdateThread.updateGraph.connect(self.PlotFlightData)

        self.content_continuous_storage = ''
        self.content_discrete_storage = ''
        self.content_string_storage = ''
        self.old_status = [0, 0, 0]
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.UpdateQuadrotorDisplay)
        self.timer.start(50)
        self.timer2 = QtCore.QTimer()
        self.timer2.timeout.connect(self.Remote_Control)
        self.timer2.start(25)

    def initUI(self):
        self.setWindowTitle('QFlight地面站')
        self.setWindowIcon(QIcon('./UI/logo.ico'))
        self.Light_toggle_btn.clicked.connect(lambda: self.Light_setting(3))
        self.Light_turn_on_btn.clicked.connect(lambda: self.Light_setting(2))
        self.Light_turn_off_btn.clicked.connect(lambda: self.Light_setting(1))
        self.Light_flash_turn_off_btn.clicked.connect(lambda: self.Light_setting(5))
        self.Light_flash_turn_on_btn.clicked.connect(lambda: self.Light_setting(4))
        self.Reboot_btn.clicked.connect(lambda: self.Send_command([0xC4, 0xC2]))
        self.Clear_flash_btn.clicked.connect(lambda: self.Send_command([0xC4, 0xC1]))
        self.Clear_gyro_offset_btn.clicked.connect(lambda: self.Send_command([0xC4, 0xC5]))
        self.Force_Stop_btn.clicked.connect(lambda: self.Send_command([0xC4, 0xC3]))
        self.Force_stop_sleep_btn.clicked.connect(lambda: self.Send_command([0xC4, 0xC4]))
        self.Verbose_turn_off_btn.clicked.connect(lambda: self.Verbose_toggle(False))
        self.Verbose_turn_on_btn.clicked.connect(lambda: self.Verbose_toggle(True))
        self.Verbose_switch_btn.clicked.connect(lambda: self.Send_command([0xC5, 0xC1]))
        self.ComPortComboBox.addItems(Communication.Get_Useful_Com())
        self.Open_com_btn.clicked.connect(self.Open_com)
        self.Open_com_link.clicked.connect(self.Open_com)
        self.Refresh_com_btn.clicked.connect(self.Refresh_com)
        self.Flash_write_in_btn.clicked.connect(self.Flash_Write_In)
        self.PID_write_in_btn.clicked.connect(self.start_PID_Write_In)
        self.PID_read_in_btn.clicked.connect(self.start_PID_Read_In)
        self.PID_default_btn.clicked.connect(self.PID_Write_defualt)
        self.PID_write_in_btn.setDisabled(True)
        self.Update_PID_Qline_Status(True)
        self.controlToggle.clicked.connect(self.Control_Toggle)
        self.Com_pause_toggle.clicked.connect(self.Toggle_ComPause)
        self.Com_clear_display_btn.clicked.connect(self.Com_Clear)
        self.Com_display_toggle.clicked.connect(self.Com_Continous_Toggle)
        self.Com_is_hex_checkbox.clicked.connect(self.Com_hex_mode_Toggle)
        self.pitch_curve = self.flightDataMonitorWidget.plot(pen='r', name='pitch')
        self.roll_curve = self.flightDataMonitorWidget.plot(pen='g', name='roll')
        self.yaw_curve = self.flightDataMonitorWidget.plot(pen='b', name='yaw')
        self.acc_x_curve = self.flightDataMonitorWidget.plot(pen='r', name='acc_x')
        self.acc_y_curve = self.flightDataMonitorWidget.plot(pen='g', name='acc_y')
        self.acc_z_curve = self.flightDataMonitorWidget.plot(pen='b', name='acc_z')
        self.gyro_x_curve = self.flightDataMonitorWidget.plot(pen='r', name='gyro_x')
        self.gyro_y_curve = self.flightDataMonitorWidget.plot(pen='g', name='gyro_y')
        self.gyro_z_curve = self.flightDataMonitorWidget.plot(pen='b', name='gyro_z')
        self.curve10 = self.flightDataMonitorWidget.plot(pen='r', name='y1')
        self.curve11 = self.flightDataMonitorWidget.plot(pen='g', name='y2')
        self.curve12 = self.flightDataMonitorWidget.plot(pen='b', name='y3')
        self.curveDisplayToggle.clicked.connect(self.Toggle_CurveDisplay)
        self.flightDataMonitorWidget.setBackground((255, 255, 255))

        self.PID_CtrlMode_Toggle.clicked.connect(self.PID_Toggle_ControlSetting)
        self.PID_ReportMode_Toggle.clicked.connect(self.PID_Toggle_ReportMode)
        self.Enable_PID_OutReport_Button.clicked.connect(self.Start_PID_OutReport)
        self.Buzzer_Status.valueChanged.connect(self.Buzzer_Status_Update)
        self.BuzzerStatusSendButton.clicked.connect(self.Buzzer_Status_Send)
        self.FX_Mode_Button.clicked.connect(self.Set_To_FX_Mode)
        self.Manual_Mode_Button.clicked.connect(self.Set_To_Manual_Mode)
        self.Send_Target_Height_Button.clicked.connect(self.Send_Target_Height)
        self.Position_Report_Button.clicked.connect(self.Open_Position_Report)
        self.accCalibrateButton.clicked.connect(self.Acc_Calibration_Start)
        self.accSetOKButton.clicked.connect(self.Acc_Prepare_Ready)
        self.Clear_Remote_Control_Offset.clicked.connect(self.Send_Clear_Remote_Ctrl_Offset)
        self.Miss_Fire_Button.clicked.connect(self.Miss_Fire_Send)
        for i in range(len(self.PID_Output_Group)):
            self.PID_Output_Group[i].setReadOnly(True)
        # for item in self.PID_Line_Edit:
        #     item.textEdited[str].connect(lambda: self.PID_Qline_Edited(item))
        # self.palette = QPalette()
        # self.palette.setColor(self.backgroundRole(), QColor(192, 253, 123))

    def Set_To_FX_Mode(self):
        self.Send_command([0xC4, 0xC7, 0x02])

    def Set_To_Manual_Mode(self):
        self.Send_command([0xC4, 0xC7, 0x01])

    def Miss_Fire_Send(self):
        self.Send_command([0xC4, 0xCB])

    def UpdateQuadrotorDisplay(self):
        self.quadrotor_mesh.resetTransform()
        self.quadrotor_mesh.translate(-25, -25, -40)
        self.quadrotor_mesh.rotate(-quadrotor.status['roll'], 1, 0, 0)
        self.quadrotor_mesh.rotate(-quadrotor.status['pitch'], 0, 1, 0)
        self.quadrotor_mesh.rotate(-quadrotor.status['yaw'], 0, 0, 1)

    def PlotFlightData(self):
        if not self.is_Curve_Display:
            return
        if self.attitudeRadioButton.isChecked():
            self.pitch_curve.setData(quadrotor.past_status['pitch'])
            self.roll_curve.setData(quadrotor.past_status['roll'])
            self.yaw_curve.setData(quadrotor.past_status['yaw'])
        else:
            quadrotor.past_status['pitch'] = []
            quadrotor.past_status['roll'] = []
            quadrotor.past_status['yaw'] = []
            self.pitch_curve.setData(quadrotor.past_status['pitch'])
            self.roll_curve.setData(quadrotor.past_status['roll'])
            self.yaw_curve.setData(quadrotor.past_status['yaw'])
        if self.accRadioButton.isChecked():
            self.acc_x_curve.setData(quadrotor.past_status['acc_x'])
            self.acc_y_curve.setData(quadrotor.past_status['acc_y'])
            self.acc_z_curve.setData(quadrotor.past_status['acc_z'])
        if self.gyroRadioButton.isChecked():
            self.gyro_x_curve.setData(quadrotor.past_status['gyro_x'])
            self.gyro_y_curve.setData(quadrotor.past_status['gyro_y'])
            self.gyro_z_curve.setData(quadrotor.past_status['gyro_z'])

    def Send_Clear_Remote_Ctrl_Offset(self):
        self.Send_command([0xC4, 0xC6])

    def Acc_Calibration_Start(self):
        self.Send_command([0xC7, 0xC1])

    def Acc_Prepare_Ready(self):
        self.Send_command([0xC7, 0xC2])

    def Acc_Data_Verify_OK(self):
        self.Send_command([0xC7, 0xC3])

    def Open_Position_Report(self):
        for i in range(3):
            self.Send_command([0xC5, 0xC5, 0X04])

    def Send_Target_Height(self):
        height_text = self.Target_Height_EditLine.text()
        try:
            height = float(height_text)
            command_list = [0xC4, 0xCA]
            command_list += self.Convert_to_2bytes(height)
            self.Send_command(command_list)
        except:
            msg_box = QMessageBox(QMessageBox.Warning, '错误', '请输入正确的参数！')
            msg_box.exec_()

    def Buzzer_Status_Update(self):
        self.Buzzer_Status_Num = self.Buzzer_Status.value()

    def Buzzer_Status_Send(self):
        self.Send_command([0xC6, 0xC1, self.Buzzer_Status.value()])

    def Start_PID_OutReport(self):
        for i in range(5):
            self.Send_command([0xC5, 0xC5, 0X03])

    def PID_Toggle_ReportMode(self):
        if self.PID_ReportMode == 0:
            self.PID_ReportMode = 1
        else:
            self.PID_ReportMode = 0
        for i, item in enumerate(self.PID_OutLabel_Group):
            item.setText(self.PID_OutLabelName_Group[self.PID_ReportMode][i])
        for i in range(5):
            self.Send_command([0xC5, 0xC6, self.PID_ReportMode])
        while quadrotor.pid_out_report_mode != self.PID_ReportMode:
            self.Send_command([0xC5, 0xC5, 0X03])
            self.Send_command([0xC5, 0xC6, self.PID_ReportMode])

    def PID_Toggle_ControlSetting(self):
        if self.PID_CtrlMode == 0:
            self.PID_CtrlMode = 1
        else:
            self.PID_CtrlMode = 0
        for i, item in enumerate(self.PID_CtrlLabel_Group):
            item.setText(self.PID_CtrlLabelName_Group[self.PID_CtrlMode][i])
        if self.PID_Line_Edit[0].text() != '/':
            self.start_PID_Read_In()

    def Remote_Control(self):
        if self.is_Com_open and self.is_GrondStation_control:
            throttle = self.throttleSlider.value() + 1
            pitch = self.pitchSlider.value() + 1
            roll = self.rollSlider.value() + 1
            yaw = self.yawSlider.value() + 1
            command = [0XC1, throttle, yaw, pitch, roll]
            quadrotor.send_command(command, self.Engine)

    def Control_Toggle(self):
        self.is_GrondStation_control = self.controlToggle.isChecked()

    def Toggle_CurveDisplay(self):
        self.is_Curve_Display = not self.is_Curve_Display
        if not self.is_Curve_Display:
            self.curveDisplayToggle.setText("开始显示")
        else:
            self.curveDisplayToggle.setText("暂停显示")

    def Toggle_ComPause(self):
        self.is_Com_puase = not self.is_Com_puase
        if self.is_Com_puase:
            self.Com_pause_toggle.setText("开始接收")
        else:
            self.Com_pause_toggle.setText("暂停接收")

    def Com_hex_mode_Toggle(self):
        self.is_Com_hex = self.Com_is_hex_checkbox.isChecked()
        if self.is_Com_hex:
            self.Com_display_toggle.setDisabled(False)
        else:
            self.Com_display_toggle.setDisabled(True)

    def Com_Clear(self):
        self.comTextBrowser.clear()
        self.content_continuous_storage = ""
        self.content_discrete_storage = ""
        self.content_string_storage = ""

    def Com_Continous_Toggle(self):
        self.is_Com_continous = not self.is_Com_continous
        self.is_Com_updating = False
        if self.is_Com_continous:
            self.Com_display_toggle.setText("离散显示")
        else:
            self.Com_display_toggle.setText("连续显示")
        self.Update_TextBrowser_All()
        self.is_Com_updating = True

    def Update_TextBrowser_All(self):
        content = self.comTextBrowser.toPlainText()
        if not content == "":
            if self.is_Com_hex:
                if self.is_Com_continous:
                    self.comTextBrowser.setText(self.content_continuous_storage)
                else:
                    self.comTextBrowser.setText(self.content_discrete_storage)
            else:
                self.comTextBrowser.setText(self.content_string_storage)
            self.Change_cursor()

    def Update_TextBrowser(self, list):
        if self.is_Com_open and not self.is_Com_puase:
            temp = ''
            for item in list:
                if item <= 15:
                    temp += '0'
                temp += hex(item)[2:] + " "
            str = temp
            str2 = str.replace(" ", "")
            try:
                string_temp = bytes(list).decode()
                self.content_string_storage += string_temp
            except:
                string_temp = ''
            self.content_continuous_storage += str2
            self.content_discrete_storage += str
            content = self.comTextBrowser.toPlainText()

            if len(content) >= 100000:
                self.Com_Clear()

            if self.is_Com_updating:
                if self.is_Com_hex:
                    if self.is_Com_continous:
                        self.comTextBrowser.insertPlainText(str2)
                    else:
                        self.comTextBrowser.insertPlainText(str)
                else:
                    if not string_temp == '':
                        self.comTextBrowser.insertPlainText(string_temp)
                self.Change_cursor()

    def Change_cursor(self):
        self.comTextBrowser.ensureCursorVisible()  # 游标可用
        cursor = self.comTextBrowser.textCursor()  # 设置游标
        pos = len(self.comTextBrowser.toPlainText())  # 获取文本尾部的位置
        cursor.setPosition(pos)  # 游标位置设置为尾部
        self.comTextBrowser.setTextCursor(cursor)  # 滚动到游标位置

    # def PID_Qline_Edited(self, PID_Qline):
    #     try:
    #         number = float(PID_Qline.text())
    #     except:
    #         PID_Qline.setPalette(self.palette)
    #     PID_Qline.setPalette(self.palette)

    def Convert_to_2bytes(self, number):
        number *= 100
        number = int(number)
        byte_array = [(number & 0x0000ff00) >> 8,
                      number & 0x000000ff]
        return byte_array

    def Convert_to_bytes(self, number):
        number *= 10000
        number = int(number)
        print(number)
        byte_array = [(number & 0xff000000) >> 24, (number & 0x00ff0000) >> 16, (number & 0x0000ff00) >> 8,
                      number & 0x000000ff]
        return byte_array

    def Flash_Write_In(self):
        if self.is_Com_open:
            for i in range(10):
                self.Verbose_toggle(False)
                time.sleep(0.05)
            if self.Flash_target_index.text().replace(" ", "") == "" or self.Flash_target_value.text().replace(" ",
                                                                                                               "") == "":
                msg_box = QMessageBox(QMessageBox.Warning, '错误', '参数不能为空！')
                msg_box.exec_()
                return
            try:
                index = int(self.Flash_target_index.text())
                if index < 0 or index > 57:
                    msg_box = QMessageBox(QMessageBox.Warning, '错误', '引索值超出允许范围！')
                    msg_box.exec_()
                    return
                value = float(self.Flash_target_value.text())
                print(value)
                if index >= 19:
                    value /= 10000
                byte_array = self.Convert_to_bytes(value)
                command = [0xC2, 0xC4, index]
                command.extend(byte_array)
                self.Send_command(command)
                time.sleep(0.5)
                self.start_PID_Read_In()
            except:
                msg_box = QMessageBox(QMessageBox.Warning, '错误', '请输入正确的参数！')
                msg_box.exec_()

        else:
            self.comWarningDialog()

    def PID_Write_defualt(self):
        if self.is_Com_open:
            self.Send_command([0xC4, 0xC1])
            time.sleep(0.5)
            self.start_PID_Read_In()
        else:
            self.comWarningDialog()

    def start_PID_Write_In(self):
        if self.is_Com_open:
            _thread.start_new_thread(self.PID_Write_In, ())
        else:
            self.comWarningDialog()

    def PID_Write_In(self):
        if self.is_Com_open:
            for i in range(10):
                self.Verbose_toggle(False)
                time.sleep(0.05)
            values = []
            try:
                for item in self.PID_Line_Edit:
                    values.append(float(item.text()))
            except:
                msg_box = QMessageBox(QMessageBox.Warning, '错误', '请输入正确的参数！')
                msg_box.exec_()
            command = [0xC2, 0xC5]
            real_values = []
            for i, value in enumerate(values):
                if (i + 1) >= 19:
                    value /= 10000
                    if value * 10000 - quadrotor.pid_parameter[i] == 0:
                        continue
                else:
                    if value - quadrotor.pid_parameter[i] == 0:
                        continue
                real_values.append({i + 1: value})
                command.append(i + 1 + 24 * self.PID_CtrlMode)
                command.extend(self.Convert_to_bytes(value))
            if len(real_values) <= 0:
                return
            self.Send_command(command)
            time.sleep(0.5)
            for i in range(5):
                self.Verbose_toggle(True)
                time.sleep(0.1)
            # self.start_PID_Read_In()
        else:
            self.comWarningDialog()

    def Verbose_toggle(self, status):
        if self.is_Com_open:
            if not status:
                self.Send_command([0xC5, 0xC2])
            else:
                self.Send_command([0xC5, 0xC3])
        else:
            self.comWarningDialog()

    def start_PID_Read_In(self):
        for item in self.PID_Line_Edit:
            item.setText("/")
        self.Update_PID_Qline_Status(True)
        if self.is_Com_open:
            _thread.start_new_thread(self.PID_Read_In, ())
        else:
            self.comWarningDialog()

    def PID_Read_In(self):
        quadrotor.pid_status = False
        self.PID_read_in_btn.setDisabled(True)
        i = 0
        while not quadrotor.pid_status:
            if not self.is_Com_open:
                self.PID_read_in_btn.setText('串口错误')
                time.sleep(1)
                self.PID_read_in_btn.setText('重新读取')
                self.PID_read_in_btn.setDisabled(False)
                return
            self.Send_command([0xC2, 0xC2, self.PID_CtrlMode])
            if i == 0:
                self.PID_read_in_btn.setText('读取中.')
            elif i == 1:
                self.PID_read_in_btn.setText('读取中..')
            elif i == 2:
                self.PID_read_in_btn.setText('读取中...')
            i = i + 1
            if i >= 3:
                i = 0
            time.sleep(0.2)
        self.Update_PID_Qline_Status(False)
        self.PID_read_in_btn.setText('读取完成')
        time.sleep(1)
        self.PID_read_in_btn.setText('重新读取')
        self.PID_read_in_btn.setDisabled(False)
        self.PID_write_in_btn.setDisabled(False)

    def Update_PID_Qline_Status(self, status):
        for i in range(len(self.PID_Line_Edit)):
            self.PID_Line_Edit[i].setReadOnly(status)

    def Refresh_com(self):
        self.ComPortComboBox.clear()
        self.ComPortComboBox.addItems(Communication.Get_Useful_Com())

    def Open_com(self):
        if not self.is_Com_open:
            if self.ComPortComboBox.currentText() == '':
                msg_box = QMessageBox(QMessageBox.Warning, '错误', '串口未选择！')
                msg_box.exec_()
                return

            self.Engine = Communication(self.ComPortComboBox.currentText(), 115200, 0.5, quadrotor)

            if self.Engine.Get_Status():
                self.is_Com_open = True
                # self.statusBar().showMessage("已连接")
                self.Open_com_btn.setText('关闭串口')
                self.Open_com_link.setText('关闭串口')
                _thread.start_new_thread(self.Engine.Recive_data, (1,))

            else:
                self.comFailedDialog()
        else:
            self.is_Com_open = False
            self.Open_com_btn.setText('打开串口')
            self.Open_com_link.setText('打开串口')
            self.Engine.Close_Engine()
            # self.statusBar().showMessage("未连接")

    def Send_command(self, command):
        temp = self.is_GrondStation_control
        self.is_GrondStation_control = False
        if self.is_Com_open:
            quadrotor.send_command(command, self.Engine)
        else:
            self.comWarningDialog()
        self.is_GrondStation_control = temp

    def Light_setting(self, para):
        if self.is_Com_open:
            toggle_light_command = [0xC3]
            if para == 1:
                toggle_light_command.append(0xC1)
            elif para == 2:
                toggle_light_command.append(0xC2)
            elif para == 3:
                toggle_light_command.append(0xC3)
            elif para == 4:
                toggle_light_command.append(0xC4)
            elif para == 5:
                toggle_light_command.append(0xC5)
            quadrotor.send_command(toggle_light_command, self.Engine)
        else:
            self.comWarningDialog()

    def comFailedDialog(self):
        msg_box = QMessageBox(QMessageBox.Warning, '警告', '串口拒绝访问！')
        msg_box.exec_()

    def comWarningDialog(self):
        msg_box = QMessageBox(QMessageBox.Warning, '警告', '串口未打开！')
        msg_box.exec_()


if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    mainWindow = MainPageWindow()
    quadrotor = Quadrotor(mainWindow)
    mainWindow.show()
    sys.exit(app.exec_())
    # if (Ret):
    #     Engine.Recive_data(0)
