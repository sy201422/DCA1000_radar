from dataclasses import dataclass, field, replace
from queue import Empty, Full
import socket
import threading as th
import time

import numpy as np

import DSP
from detection import detect_targets
from radar_runtime import (
    collapse_motion_rai,
    frame_to_radar_cube,
    integrate_rdi_channels,
    remove_static_clutter,
)
from tracking import MultiTargetTracker


DCA1000_HEADER_BYTES = 10


@dataclass(frozen=True)
class FramePacket:
    frame_id: int
    capture_ts: float
    assembled_ts: float
    iq: np.ndarray
    packets_in_frame: int
    sequence_start: int | None = None
    sequence_end: int | None = None
    byte_count_start: int | None = None
    byte_count_end: int | None = None
    udp_gap_count: int = 0
    byte_mismatch_count: int = 0
    out_of_sequence_count: int = 0
    invalid: bool = False
    invalid_reason: str = ""
    processed_ts: float | None = None
    rdi: np.ndarray | None = None
    rai: np.ndarray | None = None
    detections: tuple = field(default_factory=tuple)
    tracker_input_count: int = 0
    track_birth_blocked: bool = False
    tracker_policy: str = "full"
    confirmed_tracks: tuple = field(default_factory=tuple)
    tentative_tracks: tuple = field(default_factory=tuple)


def _put_latest(queue_object, item):
    while True:
        try:
            queue_object.put_nowait(item)
            return
        except Full:
            try:
                queue_object.get_nowait()
            except Empty:
                return


def _parse_dca1000_packet(packet_bytes):
    if len(packet_bytes) < DCA1000_HEADER_BYTES:
        return None, None, b""

    sequence_id = int.from_bytes(packet_bytes[:4], byteorder="little", signed=False)
    byte_count = int.from_bytes(packet_bytes[4:10], byteorder="little", signed=False)
    return sequence_id, byte_count, packet_bytes[DCA1000_HEADER_BYTES:]


class UdpListener(th.Thread):
    def __init__(self, name, frame_queue, data_frame_length, data_address, buff_size):
        """
        :param name: str
                        Object name

        :param frame_queue: queue object
                        A queue used to store assembled frame packets

        :param data_frame_length: int
                        Length of a single frame in int16 samples

        :param data_address: (str, int)
                        Address for binding udp stream, str for host IP address, int for host data port

        :param buff_size: int
                        Socket buffer size
        """
        th.Thread.__init__(self, name=name)
        self.frame_queue = frame_queue
        self.frame_length = data_frame_length
        self.data_address = data_address
        self.buff_size = buff_size

    def run(self):
        dt = np.dtype(np.int16).newbyteorder("<")
        sample_buffer = []
        frame_count = 0
        first_packet_logged = False

        current_capture_ts = None
        current_packet_count = 0
        current_gap_count = 0
        current_byte_mismatch_count = 0
        current_out_of_sequence_count = 0
        current_invalid = False
        current_invalid_reasons = []
        current_sequence_start = None
        current_byte_count_start = None

        last_sequence = None
        last_byte_count = None
        last_payload_size = None

        data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data_socket.bind(self.data_address)
        print("Create socket successfully")
        print("Now start data streaming")

        while True:
            packet_bytes, addr = data_socket.recvfrom(self.buff_size)
            recv_ts = time.perf_counter()
            if not first_packet_logged:
                print(f"Received first UDP packet from {addr}")
                first_packet_logged = True

            sequence_id, byte_count, payload = _parse_dca1000_packet(packet_bytes)
            if sequence_id is None:
                continue

            if current_capture_ts is None:
                current_capture_ts = recv_ts
                current_sequence_start = sequence_id
                current_byte_count_start = byte_count

            packet_gap_count = 0
            packet_byte_mismatch_count = 0
            packet_out_of_sequence_count = 0
            packet_invalid = False
            packet_invalid_reasons = []

            if last_sequence is not None:
                sequence_delta = sequence_id - last_sequence
                if sequence_delta != 1:
                    packet_out_of_sequence_count = 1
                    if sequence_delta > 1:
                        packet_gap_count = sequence_delta - 1
                    packet_invalid = True
                    packet_invalid_reasons.append("sequence")

                if last_byte_count is not None and last_payload_size is not None:
                    expected_byte_count = last_byte_count + last_payload_size
                    if byte_count != expected_byte_count:
                        packet_byte_mismatch_count = 1
                        packet_invalid = True
                        packet_invalid_reasons.append("byte_count")

            if len(payload) % dt.itemsize != 0:
                payload = payload[: len(payload) - (len(payload) % dt.itemsize)]
                packet_invalid = True
                packet_invalid_reasons.append("payload_alignment")

            if payload:
                sample_buffer.extend(np.frombuffer(payload, dtype=dt))

            current_packet_count += 1
            current_gap_count += packet_gap_count
            current_byte_mismatch_count += packet_byte_mismatch_count
            current_out_of_sequence_count += packet_out_of_sequence_count
            current_invalid = current_invalid or packet_invalid
            for reason in packet_invalid_reasons:
                if reason not in current_invalid_reasons:
                    current_invalid_reasons.append(reason)

            last_sequence = sequence_id
            last_byte_count = byte_count
            last_payload_size = len(payload)

            while len(sample_buffer) >= self.frame_length:
                frame_count += 1
                if frame_count == 1:
                    print("Received first complete radar frame")

                frame_packet = FramePacket(
                    frame_id=frame_count,
                    capture_ts=current_capture_ts if current_capture_ts is not None else recv_ts,
                    assembled_ts=recv_ts,
                    iq=np.asarray(sample_buffer[: self.frame_length], dtype=np.int16),
                    packets_in_frame=max(current_packet_count, 1),
                    sequence_start=current_sequence_start,
                    sequence_end=sequence_id,
                    byte_count_start=current_byte_count_start,
                    byte_count_end=byte_count,
                    udp_gap_count=current_gap_count,
                    byte_mismatch_count=current_byte_mismatch_count,
                    out_of_sequence_count=current_out_of_sequence_count,
                    invalid=current_invalid,
                    invalid_reason=",".join(current_invalid_reasons),
                )
                _put_latest(self.frame_queue, frame_packet)
                sample_buffer = sample_buffer[self.frame_length :]

                if sample_buffer:
                    current_capture_ts = recv_ts
                    current_packet_count = 1
                    current_gap_count = packet_gap_count
                    current_byte_mismatch_count = packet_byte_mismatch_count
                    current_out_of_sequence_count = packet_out_of_sequence_count
                    current_invalid = packet_invalid
                    current_invalid_reasons = list(packet_invalid_reasons)
                    current_sequence_start = sequence_id
                    current_byte_count_start = byte_count
                else:
                    current_capture_ts = None
                    current_packet_count = 0
                    current_gap_count = 0
                    current_byte_mismatch_count = 0
                    current_out_of_sequence_count = 0
                    current_invalid = False
                    current_invalid_reasons = []
                    current_sequence_start = None
                    current_byte_count_start = None


class DataProcessor(th.Thread):
    def __init__(
        self,
        name,
        config,
        raw_frame_queue,
        processed_frame_queue,
        detection_region,
        min_range_bin,
        max_range_bin,
        tracker: MultiTargetTracker,
        block_track_birth_on_invalid=True,
        invalid_policy=None,
    ):
        """
        :param name: str
                        Object name

        :param config: RadarRuntimeConfig
                        Parsed radar runtime config

        :param raw_frame_queue: queue object
                        A queue for access data received by UdpListener

        :param processed_frame_queue: queue object
                        A queue for processed frame packets
        """
        th.Thread.__init__(self, name=name)
        self.runtime_config = config
        self.raw_frame_queue = raw_frame_queue
        self.processed_frame_queue = processed_frame_queue
        self.detection_region = detection_region
        self.min_range_bin = min_range_bin
        self.max_range_bin = max_range_bin
        self.tracker = tracker
        self.block_track_birth_on_invalid = block_track_birth_on_invalid
        self.invalid_policy = invalid_policy or {}

    def select_tracker_input(self, frame_packet, detections):
        policy_name = "full"
        allow_track_birth = True
        tracker_detections = list(detections)
        if not frame_packet.invalid:
            return tracker_detections, allow_track_birth, policy_name

        drop_gap_threshold = int(self.invalid_policy.get("drop_gap_threshold", 0))
        drop_seq_threshold = int(self.invalid_policy.get("drop_out_of_sequence_threshold", 0))
        drop_byte_threshold = int(self.invalid_policy.get("drop_byte_mismatch_threshold", 0))
        birth_block_gap_threshold = int(self.invalid_policy.get("birth_block_gap_threshold", 0))
        birth_block_seq_threshold = int(self.invalid_policy.get("birth_block_out_of_sequence_threshold", 0))
        birth_block_byte_threshold = int(self.invalid_policy.get("birth_block_byte_mismatch_threshold", 0))

        severe_invalid = (
            frame_packet.udp_gap_count >= drop_gap_threshold > 0
            or frame_packet.out_of_sequence_count >= drop_seq_threshold > 0
            or frame_packet.byte_mismatch_count >= drop_byte_threshold > 0
        )
        moderate_invalid = (
            frame_packet.udp_gap_count >= birth_block_gap_threshold > 0
            or frame_packet.out_of_sequence_count >= birth_block_seq_threshold > 0
            or frame_packet.byte_mismatch_count >= birth_block_byte_threshold > 0
        )

        if severe_invalid:
            tracker_detections = []
            allow_track_birth = False
            policy_name = "drop"
        elif moderate_invalid and self.block_track_birth_on_invalid:
            allow_track_birth = False
            policy_name = "no_birth"

        return tracker_detections, allow_track_birth, policy_name

    def run(self):
        frame_count = 0
        while True:
            raw_frame = self.raw_frame_queue.get()
            radar_cube = frame_to_radar_cube(raw_frame.iq, self.runtime_config)
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
            tracker_detections, allow_track_birth, tracker_policy = self.select_tracker_input(raw_frame, detections)
            confirmed_tracks, tentative_tracks = self.tracker.update(
                tracker_detections,
                frame_ts=raw_frame.capture_ts,
                allow_track_birth=allow_track_birth,
            )

            processed_frame = replace(
                raw_frame,
                processed_ts=time.perf_counter(),
                rdi=rdi,
                rai=rai,
                detections=tuple(detections),
                tracker_input_count=len(tracker_detections),
                track_birth_blocked=not allow_track_birth,
                tracker_policy=tracker_policy,
                confirmed_tracks=tuple(confirmed_tracks),
                tentative_tracks=tuple(tentative_tracks),
            )

            if frame_count == 1:
                print("Generated first processed RDI/RAI frame")
                print(f"Initial detection candidates: {len(detections)}")

            _put_latest(self.processed_frame_queue, processed_frame)
