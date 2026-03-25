import os
import json
import site
from pathlib import Path
from queue import Empty, Queue
import socket
import sys
import time
from datetime import datetime
from dataclasses import dataclass

import numpy as np

# Keep pyqtgraph on the same Qt binding as the generated UI module.
os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


def _candidate_opengl_site_dirs():
    candidates = []
    user_site = site.getusersitepackages()
    if user_site:
        candidates.append(Path(user_site))

    roaming_python_root = Path.home() / 'AppData' / 'Roaming' / 'Python'
    if roaming_python_root.exists():
        for site_dir in sorted(roaming_python_root.glob('Python*/site-packages'), reverse=True):
            candidates.append(site_dir)

    unique_candidates = []
    seen = set()
    for candidate in candidates:
        candidate_str = str(candidate)
        if candidate_str in seen:
            continue
        seen.add(candidate_str)
        unique_candidates.append(candidate)
    return unique_candidates


OPENGL_AVAILABLE = False
OPENGL_IMPORT_ERROR = None
try:
    import pyqtgraph.opengl as gl
    OPENGL_AVAILABLE = True
except ModuleNotFoundError as exc:
    OPENGL_IMPORT_ERROR = exc
    if exc.name == 'OpenGL':
        for candidate_site in _candidate_opengl_site_dirs():
            candidate_str = str(candidate_site)
            if candidate_str not in sys.path and candidate_site.exists():
                sys.path.append(candidate_str)
            try:
                import pyqtgraph.opengl as gl
                OPENGL_AVAILABLE = True
                OPENGL_IMPORT_ERROR = None
                break
            except ModuleNotFoundError as retry_exc:
                OPENGL_IMPORT_ERROR = retry_exc

PROJECT_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = PROJECT_ROOT / 'tools'
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from app_layout import Ui_MainWindow
from detection import DetectionRegion
from radar_config import SerialConfig
from radar_runtime import (
    apply_cartesian_roi_to_rai,
    parse_runtime_config,
    radial_bin_limit,
)
from real_time_process import DataProcessor, UdpListener
from runtime_settings import load_runtime_settings
from tracking import MultiTargetTracker


SETTINGS = load_runtime_settings(PROJECT_ROOT)
SETTINGS_PATH = Path(SETTINGS['_settings_path'])
CONFIG_PATH = Path(SETTINGS['_config_path_resolved'])
CLI_PORT = SETTINGS['cli_port']
CLI_BAUDRATE = int(SETTINGS['cli_baudrate'])
HOST_IP = SETTINGS['network']['host_ip']
DATA_PORT = int(SETTINGS['network']['data_port'])
CONFIG_PORT = int(SETTINGS['network']['config_port'])
FPGA_IP = SETTINGS['network']['fpga_ip']
FPGA_PORT = int(SETTINGS['network']['fpga_port'])
BUFFER_SIZE = int(SETTINGS['network']['buffer_size'])
REMOVE_STATIC = bool(SETTINGS['processing']['remove_static'])
DOPPLER_GUARD_BINS = int(SETTINGS['processing']['doppler_guard_bins'])
ROI_LATERAL_M = float(SETTINGS['roi']['lateral_m'])
ROI_FORWARD_M = float(SETTINGS['roi']['forward_m'])
ROI_MIN_FORWARD_M = float(SETTINGS['roi']['min_forward_m'])
ALLOW_STRONGEST_FALLBACK = bool(SETTINGS['detection']['allow_strongest_fallback'])
TRACK_CONFIRM_HITS = int(SETTINGS['tracking']['confirm_hits'])
TRACK_MAX_MISSES = int(SETTINGS['tracking']['max_misses'])
TRACK_PROCESS_VAR = float(SETTINGS['tracking']['process_var'])
TRACK_MEASUREMENT_VAR = float(SETTINGS['tracking']['measurement_var'])
TRACK_RANGE_MEASUREMENT_SCALE = float(SETTINGS['tracking']['range_measurement_scale'])
TRACK_CONFIDENCE_MEASUREMENT_SCALE = float(SETTINGS['tracking']['confidence_measurement_scale'])
TRACK_ASSOCIATION_GATE = float(SETTINGS['tracking']['association_gate'])
TRACK_DOPPLER_ZERO_GUARD_BINS = int(SETTINGS['tracking']['doppler_zero_guard_bins'])
TRACK_DOPPLER_GATE_BINS = int(SETTINGS['tracking']['doppler_gate_bins'])
TRACK_DOPPLER_COST_WEIGHT = float(SETTINGS['tracking']['doppler_cost_weight'])
TRACK_REPORT_MISS_TOLERANCE = int(SETTINGS['tracking']['report_miss_tolerance'])
TRACK_LOST_GATE_FACTOR = float(SETTINGS['tracking']['lost_gate_factor'])
TRACK_TENTATIVE_GATE_FACTOR = float(SETTINGS['tracking']['tentative_gate_factor'])
DISPLAY_MIN_CONFIDENCE = float(SETTINGS['detection']['display_min_confidence'])
PIPELINE_QUEUE_SIZE = int(SETTINGS['pipeline']['queue_size'])
BLOCK_TRACK_BIRTH_ON_INVALID = bool(SETTINGS['pipeline']['block_track_birth_on_invalid'])
INVALID_POLICY = SETTINGS['pipeline']['invalid_policy']
DCA_CONFIG_TIMEOUT_S = float(SETTINGS['dca']['config_timeout_s'])
DCA_PACKET_SIZE_BYTES = int(SETTINGS['dca']['packet_size_bytes'])
DCA_PACKET_DELAY_US = int(SETTINGS['dca']['packet_delay_us'])
DCA_PACKET_DELAY_TICKS_PER_US = int(SETTINGS['dca']['packet_delay_ticks_per_us'])
DBSCAN_ADAPTIVE_EPS_BANDS = tuple(SETTINGS['detection']['dbscan_adaptive_eps_bands'])
DBSCAN_CLUSTER_MIN_SAMPLES = int(SETTINGS['detection']['cluster_min_samples'])
DBSCAN_CLUSTER_VELOCITY_WEIGHT = float(SETTINGS['detection']['cluster_velocity_weight'])
DETECTION_MAX_TARGETS = int(SETTINGS['detection']['max_targets'])
LOG_ROOT = PROJECT_ROOT / 'logs' / 'live_motion_viewer'
SPATIAL_VIEW_HEIGHT = int(SETTINGS['spatial_view']['height'])
SPATIAL_VIEW_Y = int(SETTINGS['spatial_view']['y'])
SPATIAL_POINT_BASE_Z_M = float(SETTINGS['spatial_view']['point_base_z_m'])
SPATIAL_POINT_CONFIDENCE_SCALE_M = float(SETTINGS['spatial_view']['point_confidence_scale_m'])
SHOW_TENTATIVE_TRACKS = bool(SETTINGS['visualization']['show_tentative_tracks'])
TENTATIVE_MIN_CONFIDENCE = float(SETTINGS['visualization']['tentative_min_confidence'])
TENTATIVE_MIN_HITS = int(SETTINGS['visualization']['tentative_min_hits'])


def send_cmd(code):
    code_map = {
        '3': (0x03).to_bytes(2, byteorder='little', signed=False),
        '5': (0x05).to_bytes(2, byteorder='little', signed=False),
        '6': (0x06).to_bytes(2, byteorder='little', signed=False),
        'B': (0x0B).to_bytes(2, byteorder='little', signed=False),
        '9': (0x09).to_bytes(2, byteorder='little', signed=False),
    }
    if code not in code_map:
        raise ValueError(f'Unsupported DCA1000 command code: {code}')

    header = (0xA55A).to_bytes(2, byteorder='little', signed=False)
    footer = (0xEEAA).to_bytes(2, byteorder='little', signed=False)
    data_size_0 = (0x00).to_bytes(2, byteorder='little', signed=False)
    data_size_6 = (0x06).to_bytes(2, byteorder='little', signed=False)
    data_fpga_config = (0x01020102031e).to_bytes(6, byteorder='big', signed=False)
    packet_delay_ticks = int(DCA_PACKET_DELAY_US * DCA_PACKET_DELAY_TICKS_PER_US)
    data_packet_config = (
        int(DCA_PACKET_SIZE_BYTES).to_bytes(2, byteorder='little', signed=False)
        + int(packet_delay_ticks).to_bytes(2, byteorder='little', signed=False)
        + (0).to_bytes(2, byteorder='little', signed=False)
    )

    if code in ('9', '5', '6'):
        return header + code_map[code] + data_size_0 + footer

    if code == '3':
        return header + code_map[code] + data_size_6 + data_fpga_config + footer

    return header + code_map[code] + data_size_6 + data_packet_config + footer


@dataclass(frozen=True)
class DcaResponse:
    command_code: int
    status: int
    payload: bytes


def parse_dca_response(response_bytes):
    if len(response_bytes) < 8:
        raise RuntimeError(f'DCA1000 response too short: {len(response_bytes)} bytes')

    header = int.from_bytes(response_bytes[:2], byteorder='little', signed=False)
    command_code = int.from_bytes(response_bytes[2:4], byteorder='little', signed=False)
    status = int.from_bytes(response_bytes[4:6], byteorder='little', signed=False)
    footer = int.from_bytes(response_bytes[-2:], byteorder='little', signed=False)

    if header != 0xA55A or footer != 0xEEAA:
        raise RuntimeError(
            'Invalid DCA1000 response packet '
            f'(header=0x{header:04X}, footer=0x{footer:04X})'
        )

    payload = response_bytes[6:-2]
    return DcaResponse(command_code=command_code, status=status, payload=payload)


class MotionViewer:
    def __init__(self):
        self.runtime_config = parse_runtime_config(
            CONFIG_PATH,
            remove_static=REMOVE_STATIC,
            doppler_guard_bins=DOPPLER_GUARD_BINS,
        )
        self.raw_frame_queue = Queue(maxsize=PIPELINE_QUEUE_SIZE)
        self.processed_frame_queue = Queue(maxsize=PIPELINE_QUEUE_SIZE)
        self.radar_ctrl = None
        self.sock_config = None
        self.collector = None
        self.processor = None
        self.img_rdi = None
        self.img_rai = None
        self.rdi_scatter = None
        self.rai_scatter = None
        self.rdi_tentative_scatter = None
        self.rai_tentative_scatter = None
        self.gl_view = None
        self.gl_scatter = None
        self.gl_stems = None
        self.gl_tentative_scatter = None
        self.gl_tentative_stems = None
        self.spatial_label = None
        self.app = None
        self.main_window = None
        self.ui = None
        self.update_time = time.perf_counter()
        self.waiting_for_data_logged = False
        self.stream_started_at = None
        self.first_image_logged = False
        self.frame_index = 0
        self.skipped_render_frames = 0
        self.log_root = LOG_ROOT
        self.session_dir = self.log_root / datetime.now().strftime('%Y%m%d_%H%M%S')
        self.status_log_path = self.session_dir / 'status_log.jsonl'
        self.runtime_config_path = self.session_dir / 'runtime_config.json'
        self.status_log_file = None
        self.min_range_bin = radial_bin_limit(
            self.runtime_config,
            ROI_MIN_FORWARD_M,
        ) if ROI_MIN_FORWARD_M > 0 else 0
        self.max_range_bin = radial_bin_limit(
            self.runtime_config,
            np.sqrt((ROI_LATERAL_M ** 2) + (ROI_FORWARD_M ** 2)),
        )
        self.display_range_bins = max(self.max_range_bin - self.min_range_bin, 1)
        self.detection_region = DetectionRegion(
            lateral_limit_m=ROI_LATERAL_M,
            forward_limit_m=ROI_FORWARD_M,
            min_forward_m=ROI_MIN_FORWARD_M,
            max_targets=DETECTION_MAX_TARGETS,
            allow_strongest_fallback=ALLOW_STRONGEST_FALLBACK,
            adaptive_eps_bands=DBSCAN_ADAPTIVE_EPS_BANDS,
            cluster_min_samples=DBSCAN_CLUSTER_MIN_SAMPLES,
            cluster_velocity_weight=DBSCAN_CLUSTER_VELOCITY_WEIGHT,
        )
        self.prepare_logging()

    def runtime_summary(self):
        return {
            'settings_path': str(SETTINGS_PATH),
            'cfg': str(CONFIG_PATH),
            'adc_sample': self.runtime_config.adc_sample,
            'chirp_loops': self.runtime_config.chirp_loops,
            'tx_num': self.runtime_config.tx_num,
            'rx_num': self.runtime_config.rx_num,
            'virtual_antennas': self.runtime_config.virtual_antennas,
            'remove_static': self.runtime_config.remove_static,
            'range_resolution_m': round(self.runtime_config.range_resolution_m, 4),
            'max_range_m': round(self.runtime_config.max_range_m, 2),
            'roi_lateral_m': ROI_LATERAL_M,
            'roi_forward_m': ROI_FORWARD_M,
            'roi_min_forward_m': ROI_MIN_FORWARD_M,
            'allow_strongest_fallback': ALLOW_STRONGEST_FALLBACK,
            'dbscan_adaptive_eps_bands': list(DBSCAN_ADAPTIVE_EPS_BANDS),
            'track_confirm_hits': TRACK_CONFIRM_HITS,
            'track_max_misses': TRACK_MAX_MISSES,
            'track_process_var': TRACK_PROCESS_VAR,
            'track_measurement_var': TRACK_MEASUREMENT_VAR,
            'track_range_measurement_scale': TRACK_RANGE_MEASUREMENT_SCALE,
            'track_confidence_measurement_scale': TRACK_CONFIDENCE_MEASUREMENT_SCALE,
            'track_association_gate': TRACK_ASSOCIATION_GATE,
            'track_doppler_zero_guard_bins': TRACK_DOPPLER_ZERO_GUARD_BINS,
            'track_doppler_gate_bins': TRACK_DOPPLER_GATE_BINS,
            'track_doppler_cost_weight': TRACK_DOPPLER_COST_WEIGHT,
            'pipeline_queue_size': PIPELINE_QUEUE_SIZE,
            'block_track_birth_on_invalid': BLOCK_TRACK_BIRTH_ON_INVALID,
            'invalid_policy': dict(INVALID_POLICY),
            'dca_packet_size_bytes': DCA_PACKET_SIZE_BYTES,
            'dca_packet_delay_us': DCA_PACKET_DELAY_US,
            'show_tentative_tracks': SHOW_TENTATIVE_TRACKS,
            'tentative_min_confidence': TENTATIVE_MIN_CONFIDENCE,
            'tentative_min_hits': TENTATIVE_MIN_HITS,
        }

    def prepare_logging(self):
        self.session_dir.mkdir(parents=True, exist_ok=True)
        with self.runtime_config_path.open('w', encoding='utf-8') as runtime_file:
            json.dump(self.runtime_summary(), runtime_file, indent=2)
        self.status_log_file = self.status_log_path.open('a', encoding='utf-8', buffering=1)
        print(f'Logging session to: {self.session_dir}')

    @staticmethod
    def serialize_detection(detection):
        return {
            'range_bin': int(detection.range_bin),
            'doppler_bin': int(detection.doppler_bin),
            'angle_bin': int(detection.angle_bin),
            'range_m': round(float(detection.range_m), 4),
            'angle_deg': round(float(detection.angle_deg), 3),
            'x_m': round(float(detection.x_m), 4),
            'y_m': round(float(detection.y_m), 4),
            'rdi_peak': round(float(detection.rdi_peak), 4),
            'rai_peak': round(float(detection.rai_peak), 4),
            'score': round(float(detection.score), 4),
        }

    def serialize_track(self, track):
        return {
            'track_id': int(track.track_id),
            'range_bin': int(self.range_bin_for_track(track)),
            'angle_bin': int(self.angle_bin_for_track(track)),
            'doppler_bin': int(track.doppler_bin),
            'range_m': round(float(track.range_m), 4),
            'angle_deg': round(float(track.angle_deg), 3),
            'x_m': round(float(track.x_m), 4),
            'y_m': round(float(track.y_m), 4),
            'vx_m_s': round(float(track.vx_m_s), 4),
            'vy_m_s': round(float(track.vy_m_s), 4),
            'rdi_peak': round(float(track.rdi_peak), 4),
            'rai_peak': round(float(track.rai_peak), 4),
            'score': round(float(track.score), 4),
            'confidence': round(float(track.confidence), 4),
            'age': int(track.age),
            'hits': int(track.hits),
            'misses': int(track.misses),
        }

    def log_status_snapshot(
        self,
        status_text,
        frame_packet,
        render_ts,
        skipped_frames,
        detections,
        tracker_input_count,
        display_tracks,
        tentative_tracks,
        tentative_display_tracks,
    ):
        if self.status_log_file is None:
            return

        elapsed_s = None
        if self.stream_started_at is not None:
            elapsed_s = max(frame_packet.capture_ts - self.stream_started_at, 0.0)

        capture_to_process_ms = None
        process_to_render_ms = None
        capture_to_render_ms = max((render_ts - frame_packet.capture_ts) * 1000.0, 0.0)
        if frame_packet.processed_ts is not None:
            capture_to_process_ms = max(
                (frame_packet.processed_ts - frame_packet.capture_ts) * 1000.0,
                0.0,
            )
            process_to_render_ms = max(
                (render_ts - frame_packet.processed_ts) * 1000.0,
                0.0,
            )

        record = {
            'frame_index': self.frame_index,
            'frame_id': int(frame_packet.frame_id),
            'wall_time': datetime.now().isoformat(timespec='milliseconds'),
            'elapsed_s': None if elapsed_s is None else round(elapsed_s, 4),
            'capture_ts': round(frame_packet.capture_ts, 6),
            'assembled_ts': round(frame_packet.assembled_ts, 6),
            'processed_ts': None if frame_packet.processed_ts is None else round(frame_packet.processed_ts, 6),
            'render_ts': round(render_ts, 6),
            'capture_to_process_ms': None if capture_to_process_ms is None else round(capture_to_process_ms, 3),
            'process_to_render_ms': None if process_to_render_ms is None else round(process_to_render_ms, 3),
            'capture_to_render_ms': round(capture_to_render_ms, 3),
            'packets_in_frame': int(frame_packet.packets_in_frame),
            'sequence_start': frame_packet.sequence_start,
            'sequence_end': frame_packet.sequence_end,
            'byte_count_start': frame_packet.byte_count_start,
            'byte_count_end': frame_packet.byte_count_end,
            'udp_gap_count': int(frame_packet.udp_gap_count),
            'byte_mismatch_count': int(frame_packet.byte_mismatch_count),
            'out_of_sequence_count': int(frame_packet.out_of_sequence_count),
            'invalid': bool(frame_packet.invalid),
            'invalid_reason': frame_packet.invalid_reason,
            'track_birth_blocked': bool(frame_packet.track_birth_blocked),
            'tracker_policy': frame_packet.tracker_policy,
            'skipped_render_frames': int(skipped_frames),
            'status_text': status_text,
            'candidate_count': len(detections),
            'tracker_input_count': int(tracker_input_count),
            'display_track_count': len(display_tracks),
            'tentative_track_count': len(tentative_tracks),
            'tentative_display_track_count': len(tentative_display_tracks),
            'detections': [self.serialize_detection(detection) for detection in detections],
            'display_tracks': [self.serialize_track(track) for track in display_tracks],
            'tentative_tracks': [self.serialize_track(track) for track in tentative_tracks],
            'tentative_display_tracks': [self.serialize_track(track) for track in tentative_display_tracks],
        }
        self.status_log_file.write(json.dumps(record, ensure_ascii=False) + '\n')

    def configure_dca1000(self):
        config_address = (HOST_IP, CONFIG_PORT)
        fpga_address = (FPGA_IP, FPGA_PORT)
        self.sock_config = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_config.bind(config_address)
        self.sock_config.settimeout(DCA_CONFIG_TIMEOUT_S)

        for command in ('9', '3', 'B', '5'):
            self.send_dca_command(command, fpga_address)

    def send_dca_command(self, command, fpga_address):
        expected_command_code = int(command, 16)
        self.sock_config.sendto(send_cmd(command), fpga_address)
        response_bytes, _ = self.sock_config.recvfrom(2048)
        response = parse_dca_response(response_bytes)

        if response.command_code != expected_command_code:
            raise RuntimeError(
                f'DCA1000 response command mismatch for 0x{expected_command_code:02X}: '
                f'got 0x{response.command_code:02X}'
            )
        if response.status != 0:
            raise RuntimeError(
                f'DCA1000 command 0x{expected_command_code:02X} failed '
                f'with status 0x{response.status:04X}'
            )
        time.sleep(0.05)
        return response

    def start_workers(self):
        data_address = (HOST_IP, DATA_PORT)
        self.collector = UdpListener(
            'Listener',
            self.raw_frame_queue,
            self.runtime_config.frame_length,
            data_address,
            BUFFER_SIZE,
        )
        self.processor = DataProcessor(
            'Processor',
            self.runtime_config,
            self.raw_frame_queue,
            self.processed_frame_queue,
            self.detection_region,
            self.min_range_bin,
            self.max_range_bin,
            MultiTargetTracker(
                process_var=TRACK_PROCESS_VAR,
                measurement_var=TRACK_MEASUREMENT_VAR,
                range_measurement_scale=TRACK_RANGE_MEASUREMENT_SCALE,
                confidence_measurement_scale=TRACK_CONFIDENCE_MEASUREMENT_SCALE,
                association_gate=TRACK_ASSOCIATION_GATE,
                doppler_center_bin=self.runtime_config.doppler_fft_size // 2,
                doppler_zero_guard_bins=TRACK_DOPPLER_ZERO_GUARD_BINS,
                doppler_gate_bins=TRACK_DOPPLER_GATE_BINS,
                doppler_cost_weight=TRACK_DOPPLER_COST_WEIGHT,
                min_confirmed_hits=TRACK_CONFIRM_HITS,
                max_missed_frames=TRACK_MAX_MISSES,
                report_miss_tolerance=TRACK_REPORT_MISS_TOLERANCE,
                lost_gate_factor=TRACK_LOST_GATE_FACTOR,
                tentative_gate_factor=TRACK_TENTATIVE_GATE_FACTOR,
            ),
            block_track_birth_on_invalid=BLOCK_TRACK_BIRTH_ON_INVALID,
            invalid_policy=INVALID_POLICY,
        )
        self.collector.daemon = True
        self.processor.daemon = True
        self.collector.start()
        self.processor.start()

    def open_radar(self):
        self.radar_ctrl = SerialConfig(
            name='ConnectRadar',
            CLIPort=CLI_PORT,
            BaudRate=CLI_BAUDRATE,
        )
        self.radar_ctrl.StopRadar()
        self.radar_ctrl.SendConfig(str(CONFIG_PATH))
        self.stream_started_at = time.perf_counter()
        self.waiting_for_data_logged = False
        self.first_image_logged = False
        self.frame_index = 0
        self.skipped_render_frames = 0
        self.update_figure()

    def pull_latest_processed_frame(self):
        frame_packet = self.processed_frame_queue.get_nowait()
        skipped_frames = 0
        while True:
            try:
                frame_packet = self.processed_frame_queue.get_nowait()
                skipped_frames += 1
            except Empty:
                break
        self.skipped_render_frames += skipped_frames
        return frame_packet, skipped_frames

    def update_figure(self):
        try:
            frame_packet, skipped_frames = self.pull_latest_processed_frame()
        except Empty:
            if (
                self.stream_started_at is not None
                and not self.first_image_logged
                and not self.waiting_for_data_logged
                and (time.perf_counter() - self.stream_started_at) > 8.0
            ):
                print(
                    "No processed frames yet. If the images stay black, "
                    "check LVDS streaming, DCA1000 link, and COM port settings."
                )
                self.waiting_for_data_logged = True
            QtCore.QTimer.singleShot(20, self.update_figure)
            return

        rdi = frame_packet.rdi
        rai = frame_packet.rai
        detections = list(frame_packet.detections)
        if rdi is None or rai is None:
            QtCore.QTimer.singleShot(5, self.update_figure)
            return

        if not self.first_image_logged:
            print("Displaying first processed frame")
            self.first_image_logged = True

        cropped_rdi = rdi[self.min_range_bin:self.max_range_bin, :]
        roi_rai = apply_cartesian_roi_to_rai(
            rai,
            self.runtime_config,
            lateral_limit_m=ROI_LATERAL_M,
            forward_limit_m=ROI_FORWARD_M,
            min_forward_m=ROI_MIN_FORWARD_M,
        )

        self.img_rdi.setImage(cropped_rdi.T, axisOrder='row-major')
        self.img_rai.setImage(np.flipud(roi_rai.T), axisOrder='row-major')
        render_ts = time.perf_counter()
        self.update_detection_overlay(frame_packet, detections, render_ts, skipped_frames)
        self.update_time = render_ts
        QtCore.QTimer.singleShot(1, self.update_figure)

    def update_detection_overlay(self, frame_packet, detections, render_ts, skipped_frames):
        tracker_input_count = int(frame_packet.tracker_input_count)
        confirmed_tracks = list(frame_packet.confirmed_tracks)
        tentative_tracks = list(frame_packet.tentative_tracks)
        display_tracks = [
            track for track in confirmed_tracks
            if track.misses <= TRACK_REPORT_MISS_TOLERANCE
            and track.confidence >= DISPLAY_MIN_CONFIDENCE
        ]
        display_tracks = sorted(
            display_tracks,
            key=lambda track: (track.confidence, track.score, track.hits),
            reverse=True,
        )
        rdi_points = [
            {
                'pos': (
                    self.range_bin_for_track(track) - self.min_range_bin,
                    track.doppler_bin,
                ),
            }
            for track in display_tracks
        ]
        rai_points = [
            {
                'pos': (
                    self.range_bin_for_track(track) - self.min_range_bin,
                    self.runtime_config.angle_fft_size - 1 - self.angle_bin_for_track(track),
                ),
            }
            for track in display_tracks
        ]
        tentative_display_tracks = []
        if SHOW_TENTATIVE_TRACKS:
            confirmed_track_ids = {track.track_id for track in display_tracks}
            tentative_display_tracks = [
                track for track in tentative_tracks
                if track.track_id not in confirmed_track_ids
                and track.misses == 0
                and track.hits >= TENTATIVE_MIN_HITS
                and track.confidence >= TENTATIVE_MIN_CONFIDENCE
            ]
            tentative_display_tracks = sorted(
                tentative_display_tracks,
                key=lambda track: (track.confidence, track.score, track.hits),
                reverse=True,
            )

        tentative_rdi_points = [
            {
                'pos': (
                    self.range_bin_for_track(track) - self.min_range_bin,
                    track.doppler_bin,
                ),
            }
            for track in tentative_display_tracks
        ]
        tentative_rai_points = [
            {
                'pos': (
                    self.range_bin_for_track(track) - self.min_range_bin,
                    self.runtime_config.angle_fft_size - 1 - self.angle_bin_for_track(track),
                ),
            }
            for track in tentative_display_tracks
        ]

        self.rdi_scatter.setData(rdi_points)
        self.rai_scatter.setData(rai_points)
        self.rdi_tentative_scatter.setData(tentative_rdi_points)
        self.rai_tentative_scatter.setData(tentative_rai_points)
        self.update_spatial_view(display_tracks, tentative_display_tracks)

        integrity_suffix = ''
        if frame_packet.invalid:
            integrity_suffix = (
                f' | invalid gaps={frame_packet.udp_gap_count} '
                f'seq={frame_packet.out_of_sequence_count} '
                f'byte={frame_packet.byte_mismatch_count}'
            )
            if frame_packet.track_birth_blocked:
                integrity_suffix += ' births=off'
            if frame_packet.tracker_policy == 'drop':
                integrity_suffix += ' tracker=drop'
        elif skipped_frames:
            integrity_suffix = f' | skipped={skipped_frames}'

        if display_tracks:
            lead_track = display_tracks[0]
            status_text = (
                'Candidates/Tracks: '
                f'{tracker_input_count}/{len(display_tracks)} | lead '
                f'id={lead_track.track_id} '
                f'r={lead_track.range_m:.2f}m '
                f'angle={lead_track.angle_deg:.1f}deg '
                f'x={lead_track.x_m:.2f}m '
                f'y={lead_track.y_m:.2f}m'
                f'{integrity_suffix}'
            )
            if tentative_display_tracks:
                status_text += f' | tentative={len(tentative_display_tracks)}'
            self.ui.statusbar.showMessage(status_text)
        else:
            status_text = (
                'Candidates/Tracks: '
                f'{tracker_input_count}/0 | tentative={len(tentative_display_tracks)}'
                f'{integrity_suffix}'
            )
            self.ui.statusbar.showMessage(status_text)

        self.frame_index += 1
        self.log_status_snapshot(
            status_text,
            frame_packet,
            render_ts,
            skipped_frames,
            detections,
            tracker_input_count,
            display_tracks,
            tentative_tracks,
            tentative_display_tracks,
        )

    def range_bin_for_track(self, track):
        return int(
            np.clip(
                np.argmin(np.abs(self.runtime_config.range_axis_m - track.range_m)),
                0,
                self.runtime_config.range_fft_size - 1,
            )
        )

    def angle_bin_for_track(self, track):
        angle_rad = np.radians(track.angle_deg)
        return int(
            np.clip(
                np.argmin(np.abs(self.runtime_config.angle_axis_rad - angle_rad)),
                0,
                self.runtime_config.angle_fft_size - 1,
            )
        )

    def update_spatial_view(self, display_tracks, tentative_display_tracks):
        if not OPENGL_AVAILABLE or self.gl_scatter is None or self.gl_stems is None:
            return

        if not display_tracks and not tentative_display_tracks:
            self.gl_scatter.setData(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=np.zeros((0, 4), dtype=np.float32),
                size=np.zeros((0,), dtype=np.float32),
            )
            self.gl_stems.setData(pos=np.zeros((0, 3), dtype=np.float32))
            if self.gl_tentative_scatter is not None:
                self.gl_tentative_scatter.setData(
                    pos=np.zeros((0, 3), dtype=np.float32),
                    color=np.zeros((0, 4), dtype=np.float32),
                    size=np.zeros((0,), dtype=np.float32),
                )
            if self.gl_tentative_stems is not None:
                self.gl_tentative_stems.setData(pos=np.zeros((0, 3), dtype=np.float32))
            return

        positions = []
        colors = []
        sizes = []
        stems = []
        for track in display_tracks:
            z_m = SPATIAL_POINT_BASE_Z_M + (SPATIAL_POINT_CONFIDENCE_SCALE_M * float(track.confidence))
            positions.append([track.x_m, track.y_m, z_m])
            colors.append([
                1.0,
                float(max(0.20, 0.85 - (0.55 * track.confidence))),
                0.20,
                0.95,
            ])
            sizes.append(10.0 + (8.0 * float(track.confidence)))
            stems.extend([
                [track.x_m, track.y_m, 0.0],
                [track.x_m, track.y_m, z_m],
            ])

        self.gl_scatter.setData(
            pos=np.asarray(positions, dtype=np.float32) if positions else np.zeros((0, 3), dtype=np.float32),
            color=np.asarray(colors, dtype=np.float32) if colors else np.zeros((0, 4), dtype=np.float32),
            size=np.asarray(sizes, dtype=np.float32) if sizes else np.zeros((0,), dtype=np.float32),
        )
        self.gl_stems.setData(
            pos=np.asarray(stems, dtype=np.float32) if stems else np.zeros((0, 3), dtype=np.float32),
            color=(1.0, 0.45, 0.20, 0.85),
            width=2.0,
            mode='lines',
        )

        if self.gl_tentative_scatter is None or self.gl_tentative_stems is None:
            return

        tentative_positions = []
        tentative_colors = []
        tentative_sizes = []
        tentative_stems = []
        for track in tentative_display_tracks:
            z_m = SPATIAL_POINT_BASE_Z_M + (0.75 * SPATIAL_POINT_CONFIDENCE_SCALE_M * float(track.confidence))
            tentative_positions.append([track.x_m, track.y_m, z_m])
            tentative_colors.append([1.0, 0.88, 0.25, 0.72])
            tentative_sizes.append(8.0 + (6.0 * float(track.confidence)))
            tentative_stems.extend([
                [track.x_m, track.y_m, 0.0],
                [track.x_m, track.y_m, z_m],
            ])

        self.gl_tentative_scatter.setData(
            pos=np.asarray(tentative_positions, dtype=np.float32) if tentative_positions else np.zeros((0, 3), dtype=np.float32),
            color=np.asarray(tentative_colors, dtype=np.float32) if tentative_colors else np.zeros((0, 4), dtype=np.float32),
            size=np.asarray(tentative_sizes, dtype=np.float32) if tentative_sizes else np.zeros((0,), dtype=np.float32),
        )
        self.gl_tentative_stems.setData(
            pos=np.asarray(tentative_stems, dtype=np.float32) if tentative_stems else np.zeros((0, 3), dtype=np.float32),
            color=(1.0, 0.88, 0.25, 0.65),
            width=1.5,
            mode='lines',
        )

    def build_window(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.main_window = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.main_window)
        self.main_window.resize(802, 680)

        self.ui.label.setGeometry(QtCore.QRect(110, 235, 211, 41))
        self.ui.label_2.setGeometry(QtCore.QRect(500, 235, 211, 41))
        self.ui.graphicsView.setGeometry(QtCore.QRect(30, 275, 361, 300))
        self.ui.graphicsView_2.setGeometry(QtCore.QRect(410, 275, 361, 300))
        self.ui.pushButton_start.setGeometry(QtCore.QRect(30, 600, 151, 31))
        self.ui.pushButton_exit.setGeometry(QtCore.QRect(680, 600, 91, 31))

        self.spatial_label = QtWidgets.QLabel(self.ui.centralwidget)
        self.spatial_label.setGeometry(QtCore.QRect(255, 8, 300, 32))
        spatial_font = self.ui.label.font()
        self.spatial_label.setFont(spatial_font)
        self.spatial_label.setAlignment(QtCore.Qt.AlignCenter)
        if OPENGL_AVAILABLE:
            self.spatial_label.setText('3D Spatial View')
            self.gl_view = gl.GLViewWidget(self.ui.centralwidget)
            self.gl_view.setGeometry(QtCore.QRect(30, SPATIAL_VIEW_Y, 741, SPATIAL_VIEW_HEIGHT))
            self.gl_view.setCameraPosition(distance=7.5, elevation=20, azimuth=-92)
            self.gl_view.opts['center'] = pg.Vector(0.0, ROI_FORWARD_M / 2.0, 0.3)

            ground_grid = gl.GLGridItem()
            ground_grid.setSize(x=ROI_LATERAL_M * 2.2, y=ROI_FORWARD_M * 1.1, z=0.0)
            ground_grid.setSpacing(x=0.5, y=0.5, z=0.5)
            ground_grid.translate(0.0, ROI_FORWARD_M / 2.0, 0.0)
            self.gl_view.addItem(ground_grid)

            axis_item = gl.GLAxisItem()
            axis_item.setSize(0.7, 0.7, 0.7)
            axis_item.translate(-ROI_LATERAL_M - 0.25, 0.0, 0.0)
            self.gl_view.addItem(axis_item)

            roi_outline = np.array(
                [
                    [-ROI_LATERAL_M, ROI_MIN_FORWARD_M, 0.0],
                    [ROI_LATERAL_M, ROI_MIN_FORWARD_M, 0.0],
                    [ROI_LATERAL_M, ROI_FORWARD_M, 0.0],
                    [-ROI_LATERAL_M, ROI_FORWARD_M, 0.0],
                    [-ROI_LATERAL_M, ROI_MIN_FORWARD_M, 0.0],
                ],
                dtype=np.float32,
            )
            roi_outline_item = gl.GLLinePlotItem(
                pos=roi_outline,
                color=(0.65, 0.65, 0.65, 1.0),
                width=1.5,
                antialias=True,
                mode='line_strip',
            )
            self.gl_view.addItem(roi_outline_item)

            self.gl_stems = gl.GLLinePlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=(1.0, 0.45, 0.20, 0.85),
                width=2.0,
                antialias=True,
                mode='lines',
            )
            self.gl_scatter = gl.GLScatterPlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=np.zeros((0, 4), dtype=np.float32),
                size=np.zeros((0,), dtype=np.float32),
            )
            self.gl_tentative_stems = gl.GLLinePlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=(1.0, 0.88, 0.25, 0.65),
                width=1.5,
                antialias=True,
                mode='lines',
            )
            self.gl_tentative_scatter = gl.GLScatterPlotItem(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=np.zeros((0, 4), dtype=np.float32),
                size=np.zeros((0,), dtype=np.float32),
            )
            self.gl_view.addItem(self.gl_stems)
            self.gl_view.addItem(self.gl_scatter)
            self.gl_view.addItem(self.gl_tentative_stems)
            self.gl_view.addItem(self.gl_tentative_scatter)
        else:
            self.spatial_label.setText('3D Spatial View (OpenGL unavailable)')

        self.ui.label.setText('Moving Range-Doppler')
        self.ui.label_2.setText('Moving Range-Angle')

        view_rdi = self.ui.graphicsView.addViewBox()
        view_rai = self.ui.graphicsView_2.addViewBox()
        view_rdi.setAspectLocked(False)
        view_rai.setAspectLocked(False)

        self.img_rdi = pg.ImageItem(border='w')
        self.img_rai = pg.ImageItem(border='w')
        self.rdi_scatter = pg.ScatterPlotItem(
            pen=pg.mkPen(255, 60, 60, width=2),
            brush=pg.mkBrush(0, 0, 0, 0),
            size=14,
        )
        self.rai_scatter = pg.ScatterPlotItem(
            pen=pg.mkPen(255, 60, 60, width=2),
            brush=pg.mkBrush(0, 0, 0, 0),
            size=14,
        )
        self.rdi_tentative_scatter = pg.ScatterPlotItem(
            pen=pg.mkPen(255, 205, 64, width=1.5),
            brush=pg.mkBrush(0, 0, 0, 0),
            size=11,
        )
        self.rai_tentative_scatter = pg.ScatterPlotItem(
            pen=pg.mkPen(255, 205, 64, width=1.5),
            brush=pg.mkBrush(0, 0, 0, 0),
            size=11,
        )
        lookup_table = self.build_lookup_table()
        self.img_rdi.setLookupTable(lookup_table)
        self.img_rai.setLookupTable(lookup_table)
        view_rdi.addItem(self.img_rdi)
        view_rai.addItem(self.img_rai)
        view_rdi.addItem(self.rdi_tentative_scatter)
        view_rai.addItem(self.rai_tentative_scatter)
        view_rdi.addItem(self.rdi_scatter)
        view_rai.addItem(self.rai_scatter)
        view_rdi.setRange(
            QtCore.QRectF(
                0,
                0,
                self.display_range_bins,
                self.runtime_config.doppler_fft_size,
            )
        )
        view_rai.setRange(
            QtCore.QRectF(
                0,
                0,
                self.display_range_bins,
                self.runtime_config.angle_fft_size,
            )
        )

        self.ui.pushButton_start.clicked.connect(self.open_radar)
        self.ui.pushButton_exit.clicked.connect(self.app.instance().exit)
        self.main_window.show()
        if not OPENGL_AVAILABLE and OPENGL_IMPORT_ERROR is not None:
            self.ui.statusbar.showMessage(
                f'3D view unavailable: {OPENGL_IMPORT_ERROR}'
            )
        return self.app

    @staticmethod
    def build_lookup_table():
        position = np.arange(64) / 64
        position[0] = 0
        position = np.flip(position)
        colors = np.flip(
            [
                [62, 38, 168, 255], [63, 42, 180, 255], [65, 46, 191, 255],
                [67, 50, 202, 255], [69, 55, 213, 255], [70, 60, 222, 255],
                [71, 65, 229, 255], [70, 71, 233, 255], [70, 77, 236, 255],
                [69, 82, 240, 255], [68, 88, 243, 255], [68, 94, 247, 255],
                [67, 99, 250, 255], [66, 105, 254, 255], [62, 111, 254, 255],
                [56, 117, 254, 255], [50, 123, 252, 255], [47, 129, 250, 255],
                [46, 135, 246, 255], [45, 140, 243, 255], [43, 146, 238, 255],
                [39, 150, 235, 255], [37, 155, 232, 255], [35, 160, 229, 255],
                [31, 164, 225, 255], [28, 129, 222, 255], [24, 173, 219, 255],
                [17, 177, 214, 255], [7, 181, 208, 255], [1, 184, 202, 255],
                [2, 186, 195, 255], [11, 189, 188, 255], [24, 191, 182, 255],
                [36, 193, 174, 255], [44, 195, 167, 255], [49, 198, 159, 255],
                [55, 200, 151, 255], [63, 202, 142, 255], [74, 203, 132, 255],
                [88, 202, 121, 255], [102, 202, 111, 255], [116, 201, 100, 255],
                [130, 200, 89, 255], [144, 200, 78, 255], [157, 199, 68, 255],
                [171, 199, 57, 255], [185, 196, 49, 255], [197, 194, 42, 255],
                [209, 191, 39, 255], [220, 189, 41, 255], [230, 187, 45, 255],
                [239, 186, 53, 255], [248, 186, 61, 255], [254, 189, 60, 255],
                [252, 196, 57, 255], [251, 202, 53, 255], [249, 208, 50, 255],
                [248, 214, 46, 255], [246, 220, 43, 255], [245, 227, 39, 255],
                [246, 233, 35, 255], [246, 239, 31, 255], [247, 245, 27, 255],
                [249, 251, 20, 255],
            ],
            axis=0,
        )
        color_map = pg.ColorMap(position, colors)
        return color_map.getLookupTable(0.0, 1.0, 256)

    def shutdown(self):
        fpga_address = (FPGA_IP, FPGA_PORT)
        if self.sock_config is not None:
            try:
                self.sock_config.sendto(send_cmd('6'), fpga_address)
            except OSError:
                pass
            self.sock_config.close()

        if self.radar_ctrl is not None:
            try:
                self.radar_ctrl.StopRadar()
            except OSError:
                pass

        if self.status_log_file is not None:
            self.status_log_file.close()
            self.status_log_file = None

    def run(self):
        print('Runtime config:', self.runtime_summary())
        self.configure_dca1000()
        self.start_workers()
        app = self.build_window()
        try:
            app.instance().exec_()
        finally:
            self.shutdown()


if __name__ == '__main__':
    MotionViewer().run()
