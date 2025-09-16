
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
            "ffmpeg", # runs the FFmpeg 
            "-f", "rawvideo", # tells FFmpeg the input has no container/headers; it’s just raw pixels.
            "-pix_fmt", "rgb24", # the input pixel format is RGB, 8 bits per channel (3 bytes/pixel).
            "-video_size", f"{width}x{height}", # the input video resolution is width x height.
            "-framerate", f"{target_fps}",        # the input frame rate is target_fps frames per second.
            "-i", "-", # the input comes from standard input (stdin - Python process writes bytes to stdin).
            "-c:v", "libx264", # use the x264 software encoder (H.264/AVC).
            "-preset", "ultrafast", # choose the fastest encoding settings (lowest CPU, larger bitrate).
            "-tune", "zerolatency", # minimize internal buffering/lookahead for low latency streaming.
            "-pix_fmt", "yuv420p", # set the output pixel format to YUV 4:2:0 (most compatible for H.264).
            "-g", "120", "-keyint_min", "120", #maximum GOP (group of pictures) length: put an IDR (keyframe) at most every 120 frames. (used for compression)
            "-sc_threshold", "0", # disable scene-cut keyframes, keeping GOP length constant.
            "-maxrate", "4M", "-bufsize", "8M", # limit the bitrate to 4Mbps with a 8Mbps buffer (helps with streaming).
            "-f", "rtsp", "-rtsp_transport", "tcp", # use RTSP protocol over TCP (more reliable than UDP).
            "rtsp://localhost:8554/mystream" # the RTSP server URL (make sure to match the server configuration).
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

