from fastapi import FastAPI, Request
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware


class KeyEvent(BaseModel):
    """ A key event model for FastAPI to parse incoming JSON data."""
    device: str
    key: str


class InputServer():
    """ A simple FastAPI server to handle key press and release events.
    It uses an InputManager instance to manage the state of keys."""

    def __init__(self,input_manager):
        """
        Initializes the InputServer with the given ffmpeg process and optional HTTP streaming.
        Args:
            ffmpeg_process: The ffmpeg process used for video streaming.
            stream_http (bool, optional): If True, sets up HTTP video streaming. Defaults to False.
        """

        self.app = FastAPI()
        self._im = input_manager

        # CORS: allow requests from your browser page (file:// or http://localhost:xxxx)
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],          # or restrict to e.g. ["http://127.0.0.1:5500"]
            allow_methods=["*"],
            allow_headers=["*"],
            allow_credentials=False,      # keep False if you want Access-Control-Allow-Origin: *
        )

        self._routes()

    def _press(self,event: KeyEvent):
        """ Handles a key press event by updating the InputManager.
        Args:
            event (KeyEvent): The key press event to handle.
        """
        self._im.set_key(event.device, event.key, True) # locad the key press to the set of pressed keys
        return {"status":f"{event.key} pressed on {event.device}"}

    def _release(self,event: KeyEvent):
        """ Handles a key release event by updating the InputManager.
        Args:
            event (KeyEvent): The key release event to handle.
        """
        self._im.set_key(event.device, event.key, False) # load the key release to the set of pressed keys
        return {"status":f"{event.key} released on {event.device}"}


    def _routes(self):
        """ Defines the API routes for the FastAPI application.
        (When an HTTP POST request hits /release, FastAPI routes it to self._release.)
        """
        self.app.add_api_route("/press", self._press, methods=["POST"])
        self.app.add_api_route("/release", self._release, methods=["POST"])



    def start(self, host = "0.0.0.0", port = 9000):
        """
        Starts the server using Uvicorn with the specified application.
        This method imports Uvicorn and runs the ASGI application (`self.app`) on host "0.0.0.0" and port 8000,
        with the log level set to "info". The server will be accessible from any network interface.
        Raises:
            Any exceptions raised by Uvicorn during server startup.
        """
        
        import uvicorn
        uvicorn.run(self.app, host=host, port=port, log_level="error")
