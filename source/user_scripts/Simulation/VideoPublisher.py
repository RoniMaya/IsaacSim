
import threading
import time
import subprocess 


class VideoPublisher():

    def __init__(self,video_rb, width = 640, height = 360, target_fps = 60):
        """
        Initializes the VideoPublisher with FFmpeg process and starts the encoding thread.
        Args:
            video_rb (RingBuffer): Ring buffer to hold video frames.
            width (int, optional): Width of the video frames. Defaults to 640.
            height (int, optional): Height of the video frames. Defaults to 360.
            target_fps (int, optional): Target frames per second for the video stream. Defaults to 60.
        Attributes:
            ffmpeg_process: The FFmpeg process for video streaming.
            ffmpeg_props: List of FFmpeg command-line arguments.
            target_fps (int): Target frames per second.
            height (int): Height of the video frames.
            width (int): Width of the video frames.
            video_rb (RingBuffer): Ring buffer to hold video frames."""
        self.ffmpeg_props = [
            "ffmpeg",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-video_size", f"{width}x{height}",
            "-framerate", f"{target_fps}",        # input framerate
            "-i", "-",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-g", "120", "-keyint_min", "120",
            "-sc_threshold", "0",
            "-maxrate", "4M", "-bufsize", "8M",
            "-f", "rtsp", "-rtsp_transport", "tcp",
            "rtsp://localhost:8554/mystream"
        ]
        self.target_fps = target_fps
        self.height = height
        self.width = width
        self.video_rb = video_rb
        self.ffmpeg_process = subprocess.Popen(self.ffmpeg_props, stdin=subprocess.PIPE)
        # start encoding thread - this will encode frames from the queue and send them to the RTSP stream
        encoding_thread = threading.Thread(target=self.encode_frames_from_rb, daemon=True)
        encoding_thread.start()


    def encode_frames_from_rb(self):
        """
        Continuously encodes frames from the ring buffer and sends them to the FFmpeg process.
        Args:
            ffmpeg_process: The FFmpeg process to which frames are sent.
        """
       
        last_seq = None
        frame_interval = 1.0 / float(self.target_fps)
        next_deadline = time.perf_counter()
        while True:
            frame = self.video_rb.latest()   # always returns newest frame

            if frame is not None:
                frame_number = frame.get("seq")
                if frame_number != last_seq:
                    self.ffmpeg_process.stdin.write(frame["bytes"])
                    last_seq = frame_number

            next_deadline += frame_interval
            sleep_for = next_deadline - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_deadline = time.perf_counter()

