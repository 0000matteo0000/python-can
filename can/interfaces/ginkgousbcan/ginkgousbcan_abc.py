# coding: utf-8

"""
Contains the ABC bus implementation and its documentation.
"""

import sys
import time
from ctypes import byref

from can import BusABC, util
from can.message import Message

if "--sim-ginkgo" not in sys.argv:
    from . import ControlCAN
else:
    from lib.helpers import timing

    from . import DummyControlCAN as ControlCAN
    ControlCAN.set_timing_function(timing)

"""
Ginkgo USB-CAN Interface.
"""


class GinkgoUsbCan(BusABC):
    """
    Enable basic can communication over a Ginkgo USB-CAN Interface device.
    """
    #: a string describing the underlying bus and/or channel
    channel_info = "Ginkgo USB-CAN Interface device."

    def __init__(self, device_index=0, can_channel=0, bitrate=250000, poll_interval=0.01, init=True, *args, **kwargs):
        """
        :param str device_type:
            The device type to open. Defaults to VCI_USBCAN2.
        :param int bitrate:
            Baud rate of the serial device in bit/s (default 250000).
        :param int device_index:
            Device index to open. Allows multiple devices to be connected at the same time.
        :param float poll_interval:
            Poll interval in seconds when reading messages
        """
        super().__init__(channel=can_channel, *args, **kwargs)
        self.device_type = ControlCAN.VCI_USBCAN2
        self.device_index = device_index
        self.channel = can_channel
        self.bitrate = bitrate
        self.poll_interval = poll_interval
        self.init = init
        self.timings = {
            5000: [0xBF, 0xFF],
            10000: [0x31, 0x1C],
            20000: [0x18, 0x1C],
            40000: [0x87, 0xFF],
            50000: [0x09, 0x1C],
            80000: [0x83, 0xFF],
            100000: [0x04, 0x1C],
            125000: [0x03, 0x1C],
            200000: [0x81, 0xFA],
            250000: [0x01, 0x1C],
            400000: [0x80, 0xFA],
            500000: [0x00, 0x1C],
            666000: [0x80, 0xB6],
            800000: [0x00, 0x16],
            1000000: [0x00, 0x14],
        }[self.bitrate]
        # SCAN DEVICE
        n_ret = ControlCAN.VCI_ScanDevice(1)
        if n_ret == 0:
            raise ConnectionError("GinkgoUsbCan: No device connected!")
        # # GET BOARD INFO
        # CAN_BoardInfo = ControlCAN.VCI_BOARD_INFO_EX()
        # n_ret = ControlCAN.VCI_ReadBoardInfoEx(self.device_index, byref(CAN_BoardInfo))
        # if(n_ret == ControlCAN.STATUS_ERR):
        #     raise ConnectionError("Get board info failed!")
        # print("--CAN_BoardInfo.ProductName = %s" % bytes(CAN_BoardInfo.ProductName).decode('ascii'), flush=True)
        # print("--CAN_BoardInfo.FirmwareVersion = V%d.%d.%d" % (CAN_BoardInfo.FirmwareVersion[1], CAN_BoardInfo.FirmwareVersion[2], CAN_BoardInfo.FirmwareVersion[3]), flush=True)
        # print("--CAN_BoardInfo.HardwareVersion = V%d.%d.%d" % (CAN_BoardInfo.HardwareVersion[1], CAN_BoardInfo.HardwareVersion[2], CAN_BoardInfo.HardwareVersion[3]), flush=True)
        # print("--CAN_BoardInfo.SerialNumber = ", end='', flush=True)
        # for i in range(0, len(CAN_BoardInfo.SerialNumber)):
        #     print("%02X" % CAN_BoardInfo.SerialNumber[i], end='', flush=True)
        # print("", flush=True)
        # OPEN DEVICE
        if self.init:
            n_ret = ControlCAN.VCI_OpenDevice(self.device_type, self.device_index, 0)
            if n_ret == ControlCAN.STATUS_ERR:
                raise ConnectionError("GinkgoUsbCan: Open device failed! You might need to run this as sudo. (type: {}, device_index: {}, channel: {})".format(self.device_type, self.device_index, self.channel))
        # CONFIG DEVICE
        can_init = ControlCAN.VCI_INIT_CONFIG()
        can_init.AccCode = 0x00000000
        can_init.AccMask = 0xFFFFFFFF
        can_init.Filter = 1
        can_init.Mode = 0
        can_init.Timing0 = self.timings[0]
        can_init.Timing1 = self.timings[1]
        n_ret = ControlCAN.VCI_InitCAN(self.device_type, self.device_index, self.channel, byref(can_init))
        if n_ret == ControlCAN.STATUS_ERR:
            raise ConnectionError("GinkgoUsbCan: Init device failed! (type: {}, device_index: {}, channel: {})".format(self.device_type, self.device_index, self.channel))
        # # REGISTER CALLBACK ON RECIVE
        # callback = ControlCAN.PVCI_RECEIVE_CALLBACK(self._recv_internal_callback)
        # ControlCAN.VCI_RegisterReceiveCallback(self.device_index, callback)
        # START CAN
        n_ret = ControlCAN.VCI_StartCAN(self.device_type, self.device_index, self.channel)
        if n_ret == ControlCAN.STATUS_ERR:
            raise ConnectionError("GinkgoUsbCan: Start CAN failed! (type: {}, device_index: {}, channel: {})".format(self.device_type, self.device_index, self.channel))

    def shutdown(self):
        """
        Close the device.
        """
        # # DISCONNECT READ CALLBACK
        # ControlCAN.VCI_LogoutReceiveCallback(self.device_index)
        # STOP RECEIVE CAN DATA
        ControlCAN.VCI_ResetCAN(self.device_type, self.device_index, self.channel)
        if self.init:
            # CLOSE DEVICE
            ControlCAN.VCI_CloseDevice(self.device_type, self.device_index)

    def _recv_internal(self, timeout):
        """
        Read a message from the Ginkgo USB-CAN Interface device.
        :param timeout:
            how long to wait for messages
        :returns:
            Received message and False (because not filtering has taken place).
            .. note:: msg fields corrispondence to VCI_CAN_OBJ(Structure): {
                        "arbitration_id": "ID",                # Frame ID
                        "timestamp": "TimeStamp",  # timestamp of the frame arriving, started from initialization of CAN controller
                        # ignored - "time_flag": "TimeFlag",   # if using timestamp. 1: use TimeStamp, 0: not use. TimeFlag and TimeStamp is available when the frame is recived frame
                        # ignored - "send_type": "SendType",   # send frame type. 0: normal send, 1: single send, 2: self send/receive, 3: single self send/receive
                        "is_remote_frame": "RemoteFlag",       # remote frame flag
                        "is_extended_id": "ExternFlag",        # extended frame flag
                        "dlc": "util.len2dlc(DataLen)",        # Data length(<=8),how many uint8_ts of data
                        "data": "Data",                        # text data
                        # ignored - "reserved": "Reserved",    # reserved
                    }
        :rtype:
            Tuple[can.Message, Bool]
        """
        if timeout is not None:
            end_time = time.time() + timeout
        else:
            end_time = None
        while True:
            data_num = ControlCAN.VCI_GetReceiveNum(self.device_type, self.device_index, self.channel)
            obj = ControlCAN.VCI_CAN_OBJ()
            if(data_num > 0):
                read_data_num = ControlCAN.VCI_Receive(self.device_type, self.device_index, self.channel, byref(obj), 1, int(timeout * 1000))
                if read_data_num == 0xFFFFFFFF:
                    raise ConnectionError("GinkgoUsbCan: Read CAN data failed! (type: {}, device_index: {}, channel: {})".format(self.device_type, self.device_index, self.channel))
                elif read_data_num != 1:
                    raise ConnectionError("GinkgoUsbCan: Could not read all CAN data! (type: {}, device_index: {}, channel: {})".format(self.device_type, self.device_index, self.channel))
                msg = Message(
                    channel=self.channel,
                    arbitration_id=obj.ID,
                    timestamp=obj.TimeStamp,
                    is_remote_frame=obj.RemoteFlag,
                    is_extended_id=obj.ExternFlag,
                    dlc=util.len2dlc(obj.DataLen),
                    data=bytes(obj.Data[i] for i in range(obj.DataLen))
                )
                # print("from: {} recived: ({}) {}".format(hex(obj.ID), obj.DataLen, bytes(obj.Data[i] for i in range(obj.DataLen)).hex(" ")), flush=True)
                return msg, False
            elif end_time is None or end_time > time.time():
                # Wait a short time until we try again
                time.sleep(self.poll_interval)
            else:
                break
        return None, False

    def send(self, msg, timeout=None):
        """
        Send a message over the Ginkgo USB-CAN Interface device.
        :param can.Message msg:
            Message to send.
            .. note:: msg fields corrispondence to VCI_CAN_OBJ(Structure): {
                        "arbitration_id": "ID",                # Frame ID
                        # ignored - "timestamp": "TimeStamp",  # timestamp of the frame arriving, started from initialization of CAN controller
                        # ignored - "time_flag": "TimeFlag",   # if using timestamp. 1: use TimeStamp, 0: not use. TimeFlag and TimeStamp is available when the frame is recived frame
                        "send_type": "SendType",               # send frame type. 0: normal send, 1: single send, 2: self send/receive, 3: single self send/receive
                        "is_remote_frame": "RemoteFlag",       # remote frame flag
                        "is_extended_id": "ExternFlag",        # extended frame flag
                        "dlc": "util.len2dlc(DataLen)",                      # Data length(<=8),how many uint8_ts of data
                        "data": "Data",                        # text data
                        # ignored - "reserved": "Reserved",    # reserved
                        Everithing else in msg will be ignored.
                    }
        :param timeout:
            This parameter will be ignored.
        """
        obj = ControlCAN.VCI_CAN_OBJ()
        obj.ID = msg.arbitration_id
        obj.SendType = getattr(msg, "send_type", 0)
        obj.RemoteFlag = 1 if msg.is_remote_frame else 0
        obj.ExternFlag = 1 if msg.is_extended_id else 0
        obj.DataLen = util.dlc2len(msg.dlc)
        if hasattr(msg, "dlc"):
            for j in range(msg.dlc):
                obj.Data[j] = msg.data[j]
        else:
            for j in range(min(len(msg.data), 8)):
                obj.Data[j] = msg.data[j]
        # print("id:\t", obj.ID, "\tdata:\t", data.hex(), flush=True)
        n_ret = ControlCAN.VCI_Transmit(self.device_type, self.device_index, self.channel, byref(obj), 1)
        if(n_ret == ControlCAN.STATUS_ERR):
            # if not ControlCAN.VCI_ResetCAN(self.device_type, self.device_index, self.channel):
            #     raise ConnectionError("GinkgoUsbCan: Reset CAN failed! (type: {}, device_index: {}, channel: {})".format(self.device_type, self.device_index, self.channel))
            raise ConnectionError(f"GinkgoUsbCan: Send CAN data failed! (type: {self.device_type}, device_index: {self.device_index}, channel: {self.channel})")
        # print("to: {} sent: ({}) {}".format(hex(obj.ID), obj.DataLen, bytes(obj.Data[i] for i in range(obj.DataLen)).hex(" ")), flush=True)

    def flush_buffer(self):
        ControlCAN.VCI_ClearBuffer(self.device_type, self.device_index, self.channel)

    # def _apply_filters(self, filters):
    #     """
    #     Hook for applying the filters to the underlying kernel or
    #     hardware if supported/implemented by the interface.
    #
    #     :param Iterator[dict] filters:
    #         See :meth:`~can.BusABC.set_filters` for details.
    #     """
    #     for i, filter in enumerate(filters):
    #         # [{"can_id": 0x11, "can_mask": 0x21, "extended": False}]
    #         filter = ControlCAN.VCI_FILTER_CONFIG()
    #         filter.Enable = 1
    #         filter.FilterIndex = i
    #         filter.FilterMode = 0
    #         filter.ExtFrame = 0
    #         filter.ID_Std_Ext = 0
    #         filter.ID_IDE = 0
    #         filter.ID_RTR = 0
    #         filter.MASK_Std_Ext = 0
    #         filter.MASK_IDE = 0
    #         filter.MASK_RTR = 0
    #         n_ret = ControlCAN.VCI_SetFilter(self.device_type, self.device_index, self.channel, byref(filter))
    #         if(n_ret == ControlCAN.STATUS_ERR):
    #             raise ConnectionError("Setting CAN filter failed!")
