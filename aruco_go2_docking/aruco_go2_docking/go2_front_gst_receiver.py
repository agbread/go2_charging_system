#!/usr/bin/env python3
"""
go2_front_gst_receiver
-----------------------
Shared GStreamer receiver for the Go2 built-in front camera H.264 stream
(UDP multicast 230.1.1.1:1720). Used by both go2_front_camera_node.py (ROS
bridge) and scripts/calibrate_go2_front.py so they share one capture path.

Decode path (verified on this robot — Jetson Orin Nano, L4T R35.3.1):
  DEFAULT  nvv4l2decoder ! nvvidconv ! BGRx   (hardware NVDEC)
  FALLBACK avdec_h264 ! videoconvert ! BGR    (software, only if HW init fails)

The fallback path loads GStreamer's libav plugin (libgstlibav.so), which on
this machine hits a libgomp static-TLS conflict when numpy was imported
first ("cannot allocate memory in static TLS block" → "no element
avdec_h264"). If the fallback is ever needed and fails that way, run with
LD_PRELOAD=libgomp.so.1. The nvv4l2decoder path does not use libav, so the
problem does not exist there at all.

Why not cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER): this robot's OpenCV
build has no GStreamer support (cv2.getBuildInformation() → "GStreamer:
NO"), so we drive GStreamer directly via PyGObject (gi.repository.Gst) and
pull frames from an appsink. Same pipeline string, different plumbing.
"""

import threading

import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

MULTICAST_ADDR = '230.1.1.1'
MULTICAST_PORT = 1720


def build_pipeline_str(network_interface, use_hw_decoder=True):
    src = (f'udpsrc address={MULTICAST_ADDR} port={MULTICAST_PORT} '
           f'multicast-iface={network_interface} ! '
           'application/x-rtp, media=video, encoding-name=H264 ! '
           'rtph264depay ! h264parse ! ')
    if use_hw_decoder:
        decode = ('nvv4l2decoder ! nvvidconv ! '
                  'video/x-raw,format=BGRx ! ')
    else:
        decode = ('avdec_h264 ! videoconvert ! '
                  'video/x-raw,format=BGR ! ')
    sink = 'appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
    return src + decode + sink


class Go2FrontGstReceiver:
    """Receives the Go2 front-camera stream and hands decoded BGR frames to
    `on_frame`. Runs a GLib main loop on a background thread; `on_frame` is
    called from that thread, not the caller's."""

    def __init__(self, network_interface='eth0', on_frame=None, on_error=None):
        self.network_interface = network_interface
        self.on_frame = on_frame        # callback(bgr_ndarray, width, height)
        self.on_error = on_error        # callback(str) — GStreamer bus errors/EOS
        self.using_hw_decoder = None    # set by start(): True=nvv4l2decoder, False=avdec
        self._pipeline = None
        self._loop = None
        self._thread = None
        self._lock = threading.Lock()
        self._latest_frame = None
        self._last_frame_time = None

    def start(self):
        Gst.init(None)
        try:
            self._start_pipeline(use_hw_decoder=True)
            self.using_hw_decoder = True
        except RuntimeError:
            self._teardown_pipeline()
            self._start_pipeline(use_hw_decoder=False)
            self.using_hw_decoder = False

        self._loop = GLib.MainLoop()
        self._thread = threading.Thread(target=self._loop.run, daemon=True)
        self._thread.start()

    def _start_pipeline(self, use_hw_decoder):
        pipeline_str = build_pipeline_str(self.network_interface, use_hw_decoder)
        try:
            self._pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            raise RuntimeError(f'pipeline parse failed: {e}')

        appsink = self._pipeline.get_by_name('sink')
        appsink.connect('new-sample', self._on_new_sample)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus_message)

        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('pipeline refused to start (state change FAILURE)')

    def _teardown_pipeline(self):
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None

    def stop(self):
        self._teardown_pipeline()
        if self._loop is not None and self._loop.is_running():
            self._loop.quit()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._loop = None

    def restart(self):
        self.stop()
        self.start()

    def get_latest_frame(self):
        """Returns (bgr_frame, monotonic_time) or (None, None) if nothing yet."""
        with self._lock:
            if self._latest_frame is None:
                return None, None
            return self._latest_frame.copy(), self._last_frame_time

    # ── GStreamer callbacks (run on the background GLib-loop thread) ──────────

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
        buf = sample.get_buffer()
        struct = sample.get_caps().get_structure(0)
        width = struct.get_value('width')
        height = struct.get_value('height')
        fmt = struct.get_value('format')            # 'BGRx' (HW) or 'BGR' (fallback)
        channels = 4 if fmt == 'BGRx' else 3

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape(
                (height, width, channels))
            # BGRx → BGR: drop the padding byte (memory order is B,G,R,X)
            frame = frame[:, :, :3].copy() if channels == 4 else frame.copy()
        finally:
            buf.unmap(mapinfo)

        now = GLib.get_monotonic_time() / 1e6
        with self._lock:
            self._latest_frame = frame
            self._last_frame_time = now

        if self.on_frame is not None:
            self.on_frame(frame, width, height)
        return Gst.FlowReturn.OK

    def _on_bus_message(self, bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            if self.on_error is not None:
                self.on_error(f'{err} ({debug})')
        elif message.type == Gst.MessageType.EOS:
            if self.on_error is not None:
                self.on_error('스트림 종료(EOS) — 멀티캐스트 수신이 끊겼을 수 있음')
        return True
