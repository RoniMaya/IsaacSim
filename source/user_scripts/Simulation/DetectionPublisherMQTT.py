import asyncio, json, time
from typing import Any, Dict, Optional
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from threading import Lock
import paho.mqtt.client as mqtt
import time
class DetectionPublisher():



    def __init__(self, ring_buffer, target_fps = 1,mqtt_properties = {'mqtt_host': '127.0.0.1', 
                                                                      'mqtt_port': 1883,
                                                                      'mqtt_topic': 'radar/detections',
                                                                      'mqtt_qos': 0,
                                                                      'mqtt_retain': False}):
        self._ring = ring_buffer
        self._target_fps = target_fps
        self._lock = Lock()
        self._latest_seq = None
        self._latest = None

        self.app = FastAPI(title="Radar JSON Publisher")

        # CORS (match your InputServer’s permissive setup)
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
            allow_credentials=False,
        )

        self._routes()
        self.mqtt_properties = mqtt_properties
        self._mqtt_connect()


    def _wrap_payload(self,item: Any) -> Dict[str, Any]:
        """
        Normalize outgoing message to include 'seq' and 'ts'.
        If 'item' is already a dict with 'seq', we reuse it.
        Otherwise we wrap it under {'data': item}.
        """
        if isinstance(item, dict):
            payload = dict(item)
            payload.setdefault("ts", time.time())
            payload.setdefault("seq", int(time.time_ns()))
            return payload
        else:
            return {"seq": int(time.time_ns()), "ts": time.time(), "data": item}

    def _mqtt_connect(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(self.mqtt_properties['mqtt_host'], self.mqtt_properties['mqtt_port'])
        self.mqtt_client.loop_start()


    def _mqtt_publish(self, payload: Dict[str, Any]):
        self.mqtt_client.publish(f"{self.mqtt_properties['mqtt_topic']}/status", json.dumps({'status': 'online', 'ts':time.time()}),qos = 0,retain = True,)
        while True:
            item = self._ring.latest()
            if item is not None:
                payload = self._wrap_payload(item)
                 
                seq = payload.get("seq") 
                with self._lock:
                    if seq is not None and seq == self._latest_seq:
                        # check of the detection is not new, if so, wait for a new one
                        time.sleep(self._period)
                        continue
                    self._latest_seq = seq
                
                self.mqtt_client.publish(
                    self.mqtt_properties['mqtt_topic'],
                    json.dumps(payload, separators=(",", ":")),
                    qos=self.mqtt_properties.get('mqtt_qos', 0),
                    retain=self.mqtt_properties.get('mqtt_retain', False),
                )
            time.sleep(self._period)

               



    def _sse_generator(self):
        # initial hello so client knows we're live
        yield "event: hello\ndata: {}\n\n"
        period = 1.0 / self._target_fps
        while True:
            item = self._ring.latest()  # direct read; no shared state
            payload = self._wrap_payload(item) if item is not None else {}
            yield f"data: {json.dumps(payload, separators=(',', ':'))}\n\n"
            time.sleep(period)




    # ---------- routes ----------
    
    def _routes(self):
        @self.app.get("/radar/stream")
        def radar_stream():
            headers = {
                "Cache-Control": "no-cache, no-transform",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            }
            return StreamingResponse(self._sse_generator(),
                                     media_type="text/event-stream",
                                     headers=headers)


        @self.app.get("/radar/latest")
        def radar_latest():
            item = self._ring.latest()
            if item is None:
                return JSONResponse({"status": "empty"}, status_code=204)
            return JSONResponse(self._wrap_payload(item))

    # ---------- server ----------
    def start(self, host: str = "0.0.0.0", port: int = 9050):
        import uvicorn
        uvicorn.run(self.app, host=host, port=port, log_level="info")