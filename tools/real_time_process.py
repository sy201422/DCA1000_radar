import threading as th
import numpy as np
import socket
import DSP
from detection import detect_targets
from radar_runtime import (
    collapse_motion_rai,
    frame_to_radar_cube,
    integrate_rdi_channels,
    remove_static_clutter,
)


class UdpListener(th.Thread):
    def __init__(self, name, bin_data, data_frame_length, data_address, buff_size):
        """
        :param name: str
                        Object name

        :param bin_data: queue object
                        A queue used to store adc data from udp stream

        :param data_frame_length: int
                        Length of a single frame

        :param data_address: (str, int)
                        Address for binding udp stream, str for host IP address, int for host data port

        :param buff_size: int
                        Socket buffer size
        """
        th.Thread.__init__(self, name=name)
        self.bin_data = bin_data
        self.frame_length = data_frame_length
        self.data_address = data_address
        self.buff_size = buff_size

    def run(self):
        # convert bytes to data type int16
        dt = np.dtype(np.int16)
        dt = dt.newbyteorder('<')
        # array for putting raw data
        np_data = []
        # count frame
        count_frame = 0
        first_packet_logged = False
        data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data_socket.bind(self.data_address)
        print("Create socket successfully")
        print("Now start data streaming")
        # main loop
        while True:
            data, addr = data_socket.recvfrom(self.buff_size)
            if not first_packet_logged:
                print(f"Received first UDP packet from {addr}")
                first_packet_logged = True
            data = data[10:]
            np_data.extend(np.frombuffer(data, dtype=dt))
            # while np_data length exceeds frame length, do following
            if len(np_data) >= self.frame_length:
                count_frame += 1
                if count_frame == 1:
                    print("Received first complete radar frame")
                # put one frame data into bin data array
                self.bin_data.put(np_data[0:self.frame_length])
                # remove one frame length data from array
                np_data = np_data[self.frame_length:]


class DataProcessor(th.Thread):
    def __init__(self, name, config, bin_queue, rdi_queue, rai_queue, detection_queue, detection_region, min_range_bin, max_range_bin):
        """
        :param name: str
                        Object name

        :param config: sequence of ints
                        Radar config in the order
                        [0]: samples number
                        [1]: chirps number
                        [3]: transmit antenna number
                        [4]: receive antenna number

        :param bin_queue: queue object
                        A queue for access data received by UdpListener

        :param rdi_queue: queue object
                        A queue for store RDI

        :param rai_queue: queue object
                        A queue for store RDI

        """
        th.Thread.__init__(self, name=name)
        self.runtime_config = config
        self.bin_queue = bin_queue
        self.rdi_queue = rdi_queue
        self.rai_queue = rai_queue
        self.detection_queue = detection_queue
        self.detection_region = detection_region
        self.min_range_bin = min_range_bin
        self.max_range_bin = max_range_bin

    def run(self):
        frame_count = 0
        while True:
            data = self.bin_queue.get()
            radar_cube = frame_to_radar_cube(data, self.runtime_config)
            if self.runtime_config.remove_static:
                radar_cube = remove_static_clutter(radar_cube)

            frame_count += 1
            rdi_cube = DSP.Range_Doppler(
                np.array(radar_cube, copy=True),
                mode=1,
                padding_size=[
                    self.runtime_config.doppler_fft_size,
                    self.runtime_config.range_fft_size,
                ],
            )
            rai_cube = DSP.Range_Angle(
                np.array(radar_cube, copy=True),
                mode=1,
                padding_size=[
                    self.runtime_config.doppler_fft_size,
                    self.runtime_config.range_fft_size,
                    self.runtime_config.angle_fft_size,
                ],
            )
            rdi = integrate_rdi_channels(rdi_cube)
            rai = collapse_motion_rai(
                rai_cube,
                guard_bins=self.runtime_config.doppler_guard_bins,
            )
            detections = detect_targets(
                rdi,
                rai,
                self.runtime_config,
                self.min_range_bin,
                self.max_range_bin,
                self.detection_region,
            )
            if frame_count == 1:
                print("Generated first processed RDI/RAI frame")
                print(f"Initial detection candidates: {len(detections)}")
            self.rdi_queue.put(rdi)
            self.rai_queue.put(rai)
            self.detection_queue.put(detections)
