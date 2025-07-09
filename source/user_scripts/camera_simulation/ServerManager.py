from fastapi import FastAPI, Request
from starlette.responses import StreamingResponse
from InputManager import InputManager
import asyncio
from uvicorn import Config, Server
import threading
import queue


class ServerManager:

    def __init__(self,ffmpeg_process, stream_http = False):
        """
        Initializes the ServerManager with the given ffmpeg process and optional HTTP streaming.
        Args:
            ffmpeg_process: The ffmpeg process used for video streaming.
            stream_http (bool, optional): If True, sets up HTTP video streaming. Defaults to False.
        """

        self.ffmpeg_process = ffmpeg_process
        self.input_manager = InputManager()
        self.app = FastAPI()
        self.setup_control_routes()
        self.frame_queue = queue.Queue(maxsize=1)
        if stream_http:
            self.setup_video_http()

    def setup_video_http(self):
        """
        Sets up an HTTP endpoint to stream H.264 video from FFmpeg using MPEG-TS format.
        """

        async def video_feed():
            """Streams the H.264 video feed from FFmpeg's output."""
            def generate_stream():
                while True:
                    # Read chunks of data directly from FFmpeg's stdout pipe
                    chunk = self.ffmpeg_process.stdout.read(4096)
                    if not chunk:
                        break
                    yield chunk
            # The media type for MPEG-TS is 'video/mp2t'
            return StreamingResponse(generate_stream(), media_type="video/mp2t")
        self.app.get("/video")(video_feed)



    def setup_control_routes(self):
        """
        Sets up HTTP POST routes for key press and release controls in the FastAPI application.
        """

        
        @self.app.post("/press")
        async def press_key(request: Request):
            """Adds a key to the set of pressed keys in a thread-safe manner."""
            command = (await request.body()).decode().strip().upper()
            self.input_manager.press_key(command)
            return {"status": f"{command} pressed"}

        @self.app.post("/release")
        async def release_key(request: Request):
            """Removes a key from the set of pressed keys in a thread-safe manner."""
            command = (await request.body()).decode().strip().upper()
            self.input_manager.release_key(command)
            return {"status": f"{command} released"}

    def start(self):
        """
        Starts the server using Uvicorn with the specified application.
        This method imports Uvicorn and runs the ASGI application (`self.app`) on host "0.0.0.0" and port 8000,
        with the log level set to "info". The server will be accessible from any network interface.
        Raises:
            Any exceptions raised by Uvicorn during server startup.
        """
        
        import uvicorn
        uvicorn.run(self.app, host="0.0.0.0", port=9000, log_level="info")


    def encode_frames_from_queue(self):
        """
        Continuously reads frames from the frame queue and writes them to the FFmpeg process for encoding.

        Handles queue timeouts and broken FFmpeg pipes gracefully (too funny to delete this).
        """

        while True:
            try:
                frame_bytes = self.frame_queue.get(timeout=1)
                self.ffmpeg_process.stdin.write(frame_bytes)
            except queue.Empty:
                continue
            except (BrokenPipeError, OSError):
                print("FFmpeg pipe broke. Stopping encoding thread.")
                break
        print("Frame encoding thread stopped.")

    # def add_frame_to_queue(self, frame_bytes):
    #     """
    #     Adds a frame (in bytes) to the frame queue.
    #     Args:
    #         frame_bytes (bytes): The frame data to be added to the queue.
    #     """

    #     self.frame_queue.append(frame_bytes)


    def add_frame_to_queue(self, frame_bytes):
        """
        Adds a frame to the queue, removing the oldest frame if the queue is full.
        Args:
            frame_bytes (bytes): The frame data to add to the queue.
        """
        
        if self.frame_queue.full():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass

        self.frame_queue.put_nowait(frame_bytes)
        return

