import _thread

import serial
import serial.tools.list_ports
import csv


class Quadrotor():
    def __init__(self, ui):
        self.status = {'pitch': 0, 'roll': 0, 'yaw': 0, 'alt': 0}
        self.moto_status = [0, 0, 0, 0]
        self.target_status = {'throttle': 0, 'pitch': 0, 'roll': 0, 'yaw': 0}
        self.sensor_data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.alt = 0
        self.ui = ui
        self.pid_parameter = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.pid_status = False
        self.verbose = True
        self.past_status = {'pitch': [], 'roll': [], 'yaw': [], 'acc_x': [], 'acc_y': [], 'acc_z': [], 'gyro_x': [],
                            'gyro_y': [], 'gyro_z': [], 'mag_x': [], 'mag_y': [], 'mag_z': [], 'moto1': [], 'moto2': [],
                            'moto3': [], 'moto4': []}
        self.pid_control_details = {"pid_out_pitch": {"p": 0, "i": 0, "d": 0, "total": "0"},
                                    "pid_out_roll": {"p": 0, "i": 0, "d": 0, "total": "0"},
                                    "pid_out_yaw": {"p": 0, "i": 0, "d": 0, "total": "0"},
                                    "pid_in_pitch": {"p": 0, "i": 0, "d": 0, "total": "0"},
                                    "pid_in_roll": {"p": 0, "i": 0, "d": 0, "total": "0"},
                                    "pid_in_yaw": {"p": 0, "i": 0, "d": 0, "total": "0"},
                                    }
        self.current_state = {'position': [0, 0, 0], 'velocity': [0, 0, 0]}
        self.target_state = {'position': [0, 0, 0], 'velocity': [0, 0, 0]}
        self.pid_out_report_mode = 0
        self.num_of_cali_record_streams = 0
        self.previous_cali_mode = -1
        self.acc_data_record=[]

    def data_display(self):
        if self.ui == 1:
            return
        self.ui.roll_display.setText(str(self.status['roll']))
        self.ui.pitch_display.setText(str(self.status['pitch']))
        self.ui.yaw_display.setText(str(self.status['yaw']))
        if not self.ui.is_GrondStation_control:
            self.ui.throttleSlider.setValue(self.target_status['throttle'])
            self.ui.rollSlider.setValue(self.target_status['roll'])
            self.ui.pitchSlider.setValue(self.target_status['pitch'])
            self.ui.yawSlider.setValue(self.target_status['yaw'])
        self.ui.acc_x_display.setText(str(self.sensor_data[0]))
        self.ui.acc_y_display.setText(str(self.sensor_data[1]))
        self.ui.acc_z_display.setText(str(self.sensor_data[2]))
        self.ui.gyro_x_display.setText(str(self.sensor_data[3]))
        self.ui.gyro_y_display.setText(str(self.sensor_data[4]))
        self.ui.gyro_z_display.setText(str(self.sensor_data[5]))
        self.ui.mag_x_display.setText(str(self.sensor_data[6]))
        self.ui.mag_y_display.setText(str(self.sensor_data[7]))
        self.ui.mag_z_display.setText(str(self.sensor_data[8]))
        self.ui.moto1Slider.setValue(10 * self.moto_status[0])
        self.ui.moto2Slider.setValue(10 * self.moto_status[1])
        self.ui.moto3Slider.setValue(10 * self.moto_status[2])
        self.ui.moto4Slider.setValue(10 * self.moto_status[3])
        self.ui.moto1Label.setText(str(self.moto_status[0]) + "%")
        self.ui.moto2Label.setText(str(self.moto_status[1]) + "%")
        self.ui.moto3Label.setText(str(self.moto_status[2]) + "%")
        self.ui.moto4Label.setText(str(self.moto_status[3]) + "%")
        self.ui.PID_Output_P_1.setText(str(self.pid_control_details['pid_out_pitch']['p']))
        self.ui.PID_Output_I_1.setText(str(self.pid_control_details['pid_out_pitch']['i']))
        self.ui.PID_Output_D_1.setText(str(self.pid_control_details['pid_out_pitch']['d']))
        self.ui.PID_Output_P_2.setText(str(self.pid_control_details['pid_out_roll']['p']))
        self.ui.PID_Output_I_2.setText(str(self.pid_control_details['pid_out_roll']['i']))
        self.ui.PID_Output_D_2.setText(str(self.pid_control_details['pid_out_roll']['d']))
        self.ui.PID_Output_P_3.setText(str(self.pid_control_details['pid_out_yaw']['p']))
        self.ui.PID_Output_I_3.setText(str(self.pid_control_details['pid_out_yaw']['i']))
        self.ui.PID_Output_D_3.setText(str(self.pid_control_details['pid_out_yaw']['d']))
        self.ui.PID_Output_Final_1.setText(str(self.pid_control_details['pid_out_pitch']['total']))
        self.ui.PID_Output_Final_2.setText(str(self.pid_control_details['pid_out_roll']['total']))
        self.ui.PID_Output_Final_3.setText(str(self.pid_control_details['pid_out_yaw']['total']))
        self.ui.PID_Output_P_4.setText(str(self.pid_control_details['pid_in_pitch']['p']))
        self.ui.PID_Output_P_5.setText(str(self.pid_control_details['pid_in_roll']['p']))
        self.ui.PID_Output_P_6.setText(str(self.pid_control_details['pid_in_yaw']['p']))
        self.ui.PID_Output_I_4.setText(str(self.pid_control_details['pid_in_pitch']['i']))
        self.ui.PID_Output_I_5.setText(str(self.pid_control_details['pid_in_roll']['i']))
        self.ui.PID_Output_I_6.setText(str(self.pid_control_details['pid_in_yaw']['i']))
        self.ui.PID_Output_D_4.setText(str(self.pid_control_details['pid_in_pitch']['d']))
        self.ui.PID_Output_D_5.setText(str(self.pid_control_details['pid_in_roll']['d']))
        self.ui.PID_Output_D_6.setText(str(self.pid_control_details['pid_in_yaw']['d']))
        self.ui.PID_Output_Final_4.setText(str(self.pid_control_details['pid_in_pitch']['total']))
        self.ui.PID_Output_Final_5.setText(str(self.pid_control_details['pid_in_roll']['total']))
        self.ui.PID_Output_Final_6.setText(str(self.pid_control_details['pid_in_yaw']['total']))
        self.ui.pos_x_display.setText(str(self.current_state['position'][0]))
        self.ui.pos_y_display.setText(str(self.current_state['position'][1]))
        self.ui.pos_z_display.setText(str(self.current_state['position'][2]))
        self.ui.velocity_x_display.setText(str(self.current_state['velocity'][0]))
        self.ui.velocity_y_display.setText(str(self.current_state['velocity'][1]))
        self.ui.velocity_z_display.setText(str(self.current_state['velocity'][2]))
        self.update_past_status()

    def update_past_status(self):
        self.past_status['pitch'].append(self.status['pitch'])
        self.past_status['roll'].append(self.status['roll'])
        self.past_status['yaw'].append(self.status['yaw'])
        self.past_status['acc_x'].append(self.sensor_data[0] / 700)
        self.past_status['acc_y'].append(self.sensor_data[1] / 700)
        self.past_status['acc_z'].append(self.sensor_data[2] / 700)
        self.past_status['gyro_x'].append(self.sensor_data[3] / 16.4)
        self.past_status['gyro_y'].append(self.sensor_data[4] / 16.4)
        self.past_status['gyro_z'].append(self.sensor_data[5] / 16.4)
        for key in self.past_status.keys():
            if len(self.past_status[key]) >= 100:
                self.past_status[key] = self.past_status[key][len(self.past_status) - 100:]
        self.ui.graphUpdateThread.run()

    def uint2int(self, number):
        if number > 32767:
            return number - 65535
        else:
            return number

    def justfyFloat(self, number):
        return float(int(number * 100)) / 100

    def data_precessing(self, data_buffer):
        new_data_buffer = data_buffer[2:]
        new_data_buffer = new_data_buffer[:-2]
        function = new_data_buffer[0]
        if function == 0x01:
            self.status['roll'] = ((new_data_buffer[1] << 8) | new_data_buffer[2])
            self.status['pitch'] = ((new_data_buffer[3] << 8) | new_data_buffer[4])
            self.status['yaw'] = ((new_data_buffer[5] << 8) | new_data_buffer[6])
            self.status['alt'] = (new_data_buffer[7] << 24) | (
                    (new_data_buffer[8] << 16) | ((new_data_buffer[9] << 8) | new_data_buffer[10]))
            for key in self.status.keys():
                if self.status[key] > 32767:
                    self.status[key] = self.status[key] - 65535
                self.status[key] /= 100
            # print(self.status)
            moto_temp = []
            j = 11
            for i in range(4):
                moto_temp.append((new_data_buffer[j] << 8) | new_data_buffer[j + 1])
                j += 2
            for i, item in enumerate(moto_temp):
                self.moto_status[i] = (item - 1000) / 10
            self.target_status['throttle'] = ((new_data_buffer[19] << 8) | new_data_buffer[20])
            self.target_status['yaw'] = (new_data_buffer[21] << 8) | new_data_buffer[22]
            self.target_status['roll'] = ((new_data_buffer[23] << 8) | new_data_buffer[24])
            self.target_status['pitch'] = ((new_data_buffer[25] << 8) | new_data_buffer[26])

            for key in self.target_status.keys():
                if self.target_status[key] > 32767:
                    self.target_status[key] = self.target_status[key] - 65535
                self.target_status[key] /= 100
            self.target_status['throttle'] *= 100
            self.target_status['roll'] = (50 / 10) * self.target_status['roll'] + 51
            self.target_status['pitch'] = (50 / 10) * self.target_status['pitch'] + 51
            self.target_status['yaw'] = (50 / 10) * self.target_status['yaw'] + 51
            j = 27
            for i in range(9):
                self.sensor_data[i] = (new_data_buffer[j] << 8) | new_data_buffer[j + 1]
                j += 2
            for i, item in enumerate(self.sensor_data):
                if item > 32767:
                    self.sensor_data[i] = item - 65535
            self.data_display()
        elif function == 0x03:
            constant = 50
            self.pid_out_report_mode = new_data_buffer[1]
            if self.pid_out_report_mode == 0:
                self.pid_control_details["pid_out_pitch"]["p"] = self.uint2int(
                    (new_data_buffer[2] << 8) | new_data_buffer[3]) / constant
                self.pid_control_details["pid_out_roll"]["p"] = self.uint2int(
                    (new_data_buffer[4] << 8) | new_data_buffer[5]) / constant
                self.pid_control_details["pid_out_yaw"]["p"] = self.uint2int(
                    (new_data_buffer[6] << 8) | new_data_buffer[7]) / constant
                self.pid_control_details["pid_in_pitch"]["p"] = self.uint2int(
                    (new_data_buffer[8] << 8) | new_data_buffer[9]) / constant
                self.pid_control_details["pid_in_pitch"]["i"] = self.uint2int(
                    (new_data_buffer[10] << 8) | new_data_buffer[11]) / constant
                self.pid_control_details["pid_in_pitch"]["d"] = self.uint2int(
                    (new_data_buffer[12] << 8) | new_data_buffer[13]) / constant
                self.pid_control_details["pid_in_roll"]["p"] = self.uint2int(
                    (new_data_buffer[14] << 8) | new_data_buffer[15]) / constant
                self.pid_control_details["pid_in_roll"]["i"] = self.uint2int(
                    (new_data_buffer[16] << 8) | new_data_buffer[17]) / constant
                self.pid_control_details["pid_in_roll"]["d"] = self.uint2int(
                    (new_data_buffer[18] << 8) | new_data_buffer[19]) / constant
                self.pid_control_details["pid_in_yaw"]["p"] = self.uint2int(
                    (new_data_buffer[20] << 8) | new_data_buffer[21]) / constant
                self.pid_control_details["pid_in_yaw"]["i"] = self.uint2int(
                    (new_data_buffer[22] << 8) | new_data_buffer[23]) / constant
                self.pid_control_details["pid_in_yaw"]["d"] = self.uint2int(
                    (new_data_buffer[24] << 8) | new_data_buffer[25]) / constant
            elif self.pid_out_report_mode == 1:
                self.pid_control_details["pid_out_pitch"]["p"] = self.uint2int(
                    (new_data_buffer[2] << 8) | new_data_buffer[3]) / constant
                self.pid_control_details["pid_out_pitch"]["i"] = self.uint2int(
                    (new_data_buffer[4] << 8) | new_data_buffer[5]) / constant
                self.pid_control_details["pid_out_pitch"]["d"] = self.uint2int(
                    (new_data_buffer[6] << 8) | new_data_buffer[7]) / constant
                self.pid_control_details["pid_out_roll"]["p"] = self.uint2int(
                    (new_data_buffer[8] << 8) | new_data_buffer[9]) / constant
                self.pid_control_details["pid_out_roll"]["i"] = self.uint2int(
                    (new_data_buffer[10] << 8) | new_data_buffer[11]) / constant
                self.pid_control_details["pid_out_roll"]["d"] = self.uint2int(
                    (new_data_buffer[12] << 8) | new_data_buffer[13]) / constant
                self.pid_control_details["pid_out_yaw"]["p"] = self.uint2int(
                    (new_data_buffer[14] << 8) | new_data_buffer[15]) / constant
                self.pid_control_details["pid_out_yaw"]["i"] = self.uint2int(
                    (new_data_buffer[16] << 8) | new_data_buffer[17]) / constant
                self.pid_control_details["pid_out_yaw"]["d"] = self.uint2int(
                    (new_data_buffer[18] << 8) | new_data_buffer[19]) / constant
                self.pid_control_details["pid_in_pitch"]["p"] = self.uint2int(
                    (new_data_buffer[20] << 8) | new_data_buffer[21]) / constant
                self.pid_control_details["pid_in_pitch"]["i"] = self.uint2int(
                    (new_data_buffer[22] << 8) | new_data_buffer[23]) / constant
                self.pid_control_details["pid_in_pitch"]["d"] = self.uint2int(
                    (new_data_buffer[24] << 8) | new_data_buffer[25]) / constant
                self.pid_control_details["pid_in_roll"]["p"] = self.uint2int(
                    (new_data_buffer[26] << 8) | new_data_buffer[27]) / constant
                self.pid_control_details["pid_in_roll"]["i"] = self.uint2int(
                    (new_data_buffer[28] << 8) | new_data_buffer[29]) / constant
                self.pid_control_details["pid_in_roll"]["d"] = self.uint2int(
                    (new_data_buffer[30] << 8) | new_data_buffer[31]) / constant
                self.pid_control_details["pid_in_yaw"]["p"] = self.uint2int(
                    (new_data_buffer[32] << 8) | new_data_buffer[33]) / constant
                self.pid_control_details["pid_in_yaw"]["i"] = self.uint2int(
                    (new_data_buffer[34] << 8) | new_data_buffer[35]) / constant
                self.pid_control_details["pid_in_yaw"]["d"] = self.uint2int(
                    (new_data_buffer[36] << 8) | new_data_buffer[37]) / constant

            for key in self.pid_control_details.keys():
                self.pid_control_details[key]['total'] = self.justfyFloat(
                    self.pid_control_details[key]['p'] + self.pid_control_details[key]['i'] +
                    self.pid_control_details[key]['d'])
            self.data_display()
        elif function == 0x05:
            j = 1
            for i in range(24):
                if i < 18:
                    self.pid_parameter[i] = float((new_data_buffer[j] << 24) | (
                            (new_data_buffer[j + 1] << 16) | (
                            (new_data_buffer[j + 2] << 8) | new_data_buffer[j + 3]))) / 10000
                else:
                    self.pid_parameter[i] = float((new_data_buffer[j] << 24) | (
                            (new_data_buffer[j + 1] << 16) | (
                            (new_data_buffer[j + 2] << 8) | new_data_buffer[j + 3])))
                j += 4
            for i, item in enumerate(self.ui.PID_Line_Edit):
                item.setText(str(self.pid_parameter[i]))
            self.pid_status = True
        elif function == 0x06:
            constant = 100
            self.current_state['position'][0] = self.uint2int(
                (new_data_buffer[1] << 8) | new_data_buffer[2]) / constant
            self.current_state['position'][1] = self.uint2int(
                (new_data_buffer[3] << 8) | new_data_buffer[4]) / constant
            self.current_state['position'][2] = self.uint2int(
                (new_data_buffer[5] << 8) | new_data_buffer[5]) / constant
            self.current_state['velocity'][0] = self.uint2int(
                (new_data_buffer[7] << 8) | new_data_buffer[8]) / constant
            self.current_state['velocity'][1] = self.uint2int(
                (new_data_buffer[9] << 8) | new_data_buffer[10]) / constant
            self.current_state['velocity'][2] = self.uint2int(
                (new_data_buffer[11] << 8) | new_data_buffer[12]) / constant
            self.target_state['position'][0] = self.uint2int(
                (new_data_buffer[13] << 8) | new_data_buffer[14]) / constant
            self.target_state['position'][1] = self.uint2int(
                (new_data_buffer[15] << 8) | new_data_buffer[16]) / constant
            self.target_state['position'][2] = self.uint2int(
                (new_data_buffer[17] << 8) | new_data_buffer[18]) / constant
            self.target_state['velocity'][0] = self.uint2int(
                (new_data_buffer[19] << 8) | new_data_buffer[20]) / constant
            self.target_state['velocity'][1] = self.uint2int(
                (new_data_buffer[21] << 8) | new_data_buffer[22]) / constant
            self.target_state['velocity'][2] = self.uint2int(
                (new_data_buffer[23] << 8) | new_data_buffer[24]) / constant
            self.ui.pos_x_display.setText(str(self.current_state['position'][0]))
            self.ui.pos_y_display.setText(str(self.current_state['position'][1]))
            self.ui.pos_z_display.setText(str(self.current_state['position'][2]))
            self.ui.velocity_x_display.setText(str(self.current_state['velocity'][0]))
            self.ui.velocity_y_display.setText(str(self.current_state['velocity'][1]))
            self.ui.velocity_z_display.setText(str(self.current_state['velocity'][2]))
            self.ui.Current_Velocity_X.setText(str(self.current_state['velocity'][0]))
            self.ui.Current_Velocity_Y.setText(str(self.current_state['velocity'][1]))
            self.ui.Current_Velocity_Z.setText(str(self.current_state['velocity'][2]))
            self.ui.Current_Position_X.setText(str(self.current_state['position'][0]))
            self.ui.Current_Position_Y.setText(str(self.current_state['position'][1]))
            self.ui.Current_Position_Z.setText(str(self.current_state['position'][2]))
            self.ui.Target_Velocity_X.setText(str(self.target_state['velocity'][0]))
            self.ui.Target_Velocity_Y.setText(str(self.target_state['velocity'][1]))
            self.ui.Target_Velocity_Z.setText(str(self.target_state['velocity'][2]))
            self.ui.Target_Position_X.setText(str(self.target_state['position'][0]))
            self.ui.Target_Position_Y.setText(str(self.target_state['position'][1]))
            self.ui.Target_Position_Z.setText(str(self.target_state['position'][2]))
        elif function == 0x07:
            if new_data_buffer[1] == 0xAC:
                cali_mode = new_data_buffer[2]

                if self.num_of_cali_record_streams == -1:
                    if self.previous_cali_mode == cali_mode:
                        self.ui.Send_command([0xC7, 0xC3])
                        return
                    else:
                        self.previous_cali_mode = cali_mode
                        self.num_of_cali_record_streams = 0

                data_len = len(new_data_buffer) - 4
                if data_len % 6 != 0:
                    print(str(data_len) + ": error in data length!")
                    return
                data_len /= 6

                acc_data = []
                j = 3
                for i in range(int(data_len)):
                    acc_single_data = []
                    for z in range(3):
                        num = self.uint2int(
                            (new_data_buffer[j] << 8) | new_data_buffer[j + 1])
                        j += 2
                        acc_single_data.append(num)

                    acc_data.append(acc_single_data)
                self.acc_data_record+=acc_data
                if self.num_of_cali_record_streams!=-1:
                    self.num_of_cali_record_streams += 1
                if self.num_of_cali_record_streams >= 90:
                    print(cali_mode)
                    print("finished!")
                    filename = "Cali_Mode_" + str(cali_mode) + ".csv"
                    with open('./' + filename, 'w', newline='') as file:
                        csv_writer = csv.writer(file)
                        for l in self.acc_data_record:
                            csv_writer.writerow(l)
                    self.acc_data_record=[]
                    self.num_of_cali_record_streams = -1
                    self.ui.Send_command([0xC7, 0xC3])

    def send_command(self, command_array, Engine):
        final_command_array = [0xC8]
        final_command_array += command_array
        sum = 0
        for item in final_command_array:
            sum += item
        final_command_array.append(sum & 0x00FF)
        final_command_array.append(0xC9)
        final_command = bytearray(final_command_array)
        print(final_command_array)
        Engine.Send_data(final_command)


class Communication():
    # 初始化
    def __init__(self, com, bps, timeout, quadrotor):
        self.port = com
        self.bps = bps
        self.timeout = timeout
        self.data_buffer = []
        self.quadrotor = quadrotor
        self.status = False
        self.buffer_string = ""
        global Ret
        try:
            # 打开串口，并得到串口对象
            self.main_engine = serial.Serial(self.port, self.bps, timeout=self.timeout)
            # 判断是否打开成功
            if self.main_engine.is_open:
                Ret = True
                self.status = True
        except Exception as e:
            self.status = False
            self.quadrotor.ui.Open_com_btn.setText("打开串口")
            self.quadrotor.ui.Open_com_link.setText("打开串口")
            print("---异常---：", e)

    def Get_Status(self):
        try:
            return self.main_engine.is_open
        except:
            return False

    # 打印设备基本信息
    def Print_Name(self):
        print(self.main_engine.name)  # 设备名字
        print(self.main_engine.port)  # 读或者写端口
        print(self.main_engine.baudrate)  # 波特率
        print(self.main_engine.bytesize)  # 字节大小
        print(self.main_engine.parity)  # 校验位
        print(self.main_engine.stopbits)  # 停止位
        print(self.main_engine.timeout)  # 读超时设置
        print(self.main_engine.writeTimeout)  # 写超时
        print(self.main_engine.xonxoff)  # 软件流控
        print(self.main_engine.rtscts)  # 软件流控
        print(self.main_engine.dsrdtr)  # 硬件流控
        print(self.main_engine.interCharTimeout)  # 字符间隔超时

    # 打开串口
    def Open_Engine(self):
        self.status = True
        self.main_engine.open()

    # 关闭串口
    def Close_Engine(self):
        self.status = False
        self.main_engine.close()

    # 打印可用串口列表
    @staticmethod
    def Get_Useful_Com():
        device_list = list(serial.tools.list_ports.comports())
        port_list = []
        for item in device_list:
            port_list.append(item.device)
        port_list.reverse()
        return port_list

    # 接收指定大小的数据
    # 从串口读size个字节。如果指定超时，则可能在超时后返回较少的字节；如果没有指定超时，则会一直等到收完指定的字节数。
    def Read_Size(self, size):
        return self.main_engine.read(size=size)

    # 接收一行数据
    # 使用readline()时应该注意：打开串口时应该指定超时，否则如果串口没有收到新行，则会一直等待。
    # 如果没有超时，readline会报异常。
    def Read_Line(self):
        return self.main_engine.readline()

    # 发数据
    def Send_data(self, data):
        self.main_engine.write(data)

    # 更多示例
    # self.main_engine.write(chr(0x06).encode("utf-8"))  # 十六制发送一个数据
    # print(self.main_engine.read().hex())  #  # 十六进制的读取读一个字节
    # print(self.main_engine.read())#读一个字节
    # print(self.main_engine.read(10).decode("gbk"))#读十个字节
    # print(self.main_engine.readline().decode("gbk"))#读一行
    # print(self.main_engine.readlines())#读取多行，返回列表，必须匹配超时（timeout)使用
    # print(self.main_engine.in_waiting)#获取输入缓冲区的剩余字节数
    # print(self.main_engine.out_waiting)#获取输出缓冲区的字节数
    # print(self.main_engine.readall())#读取全部字符。

    # 接收数据
    # 一个整型数据占两个字节
    # 一个字符占一个字节

    def Recive_data(self, way):
        # 循环接收数据，此为死循环，可用线程实现
        buffer = []
        while True:
            if self.status:
                try:
                    # 一个字节一个字节的接收
                    if self.main_engine.in_waiting:
                        if (way == 0):
                            temp = 0
                            toggle = False
                            for i in range(self.main_engine.in_waiting):
                                # print("接收ascii数据："+str(self.Read_Size(1)))
                                data1 = self.Read_Size(1).hex()  # 转为十六进制
                                data2 = int(data1, 16)  # 转为十进制print("收到数据十六进制："+data1+"  收到数据十进制："+str(data2))

                                if data2 == 0xcc and temp == 0xcc and not toggle:
                                    toggle = True
                                    self.data_buffer.append(0xcc)
                                if toggle:
                                    self.data_buffer.append(data2)
                                    if data2 == 0xc9:
                                        pass
                                    if data2 == 0xc9 and temp == 0xc9:
                                        toggle = False
                                        required_items = self.data_buffer[:-3]
                                        sum = self.data_buffer[-3]
                                        real_sum = 0
                                        for item in required_items:
                                            real_sum += item
                                        if real_sum & 0x00FF == sum:
                                            # for data_item in data_buffer:
                                            #     print('%x' % data_item, end='')
                                            # print(";")
                                            self.quadrotor.data_precessing(self.data_buffer)
                                        self.data_buffer.clear()
                                temp = data2

                        if (way == 1):
                            # 整体接收
                            # data = self.main_engine.read(self.main_engine.in_waiting).decode("utf-8")#方式一
                            data = self.main_engine.read_all()  # 方式二print("接收ascii数据：", data)
                            data = list(data)
                            self.buffer_string = data
                            if self.quadrotor.ui != 1:
                                self.quadrotor.ui.comUpdateThread.run()

                            buffer.extend(data)

                            if len(buffer) >= 100:
                                buffer = buffer[is_protocol_head(buffer):]
                                buffer = buffer[:get_protocol_tail(buffer)]
                                # toggle=False

                                if len(buffer) >= 50 or len(buffer) == 31 or len(buffer) == 43 or len(buffer) == 30:
                                    required_items = buffer[:-3]
                                    sum = buffer[-3]
                                    real_sum = 0
                                    for item in required_items:
                                        real_sum += item
                                    if real_sum & 0x00FF == sum:
                                        # for data_item in data_buffer:
                                        #     print('%x' % data_item, end='')
                                        # print(";")
                                        self.quadrotor.data_precessing(buffer)
                                buffer.clear()


                except Exception as e:
                    if self.quadrotor.ui != 1:
                        self.quadrotor.ui.Open_com_btn.setText("打开串口")
                        self.quadrotor.ui.Open_com_link.setText("打开串口")
                        self.quadrotor.ui.is_Com_open = False
                    self.status = False
                    # self.quadrotor.ui.statusBar().showMessage("已断开")
                    print("异常报错：", e)
            else:
                break


def is_protocol_head(array):
    item_old = 0
    for i, item in enumerate(array):
        if item_old == 204 and item == 204:
            return i - 1
        item_old = item
    return -1


def get_protocol_tail(array):
    item_old = 0
    for i, item in enumerate(array):
        if item_old == 201 and item == 201:
            return i + 1
        item_old = item
    return -1


if __name__ == '__main__':
    quadrotor = Quadrotor(1)
    Engine = Communication('com3', 115200, 0.5, quadrotor)
    Engine.status = True
    Engine.Recive_data(1)
    # a=[204,204,1,2,3,201,201,2,3]
    # print(get_protocol_tail(a))
    # print(a[:7])
