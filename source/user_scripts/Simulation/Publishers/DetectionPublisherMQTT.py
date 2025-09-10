import asyncio, json, time
from typing import Any, Dict, Optional
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from threading import Lock
from datetime import datetime, timedelta
from pytz import UTC

# import sys
# sys.path.insert(0, "/home/ronim/isaacsim/third_party/pydeps")


import paho.mqtt.client as mqtt
import time

DEFAULT_TRACKING_TIMEOUT = 10


class DetectionPublisherMQTT():

    def __init__(self, ring_buffer,radar_params, target_fps = 1,mqtt_properties = {'mqtt_host': '100.87.66.85', 
                                                                      'mqtt_port': 1883,
                                                                      'mqtt_topic': '/device/magos/magos-service/platform/',
                                                                      'mqtt_qos': 0,
                                                                      'mqtt_retain': False, 'client_id': "radar_publisher"}):
        self._ring = ring_buffer
        self._target_fps = target_fps
        self._lock = Lock()
        self._latest_seq = None
        self._latest = None
        self._period = 1.0 / max(1, int(target_fps))

        self.app = FastAPI(title="Radar JSON Publisher")

        # CORS (match your InputServer’s permissive setup)
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
            allow_credentials=False,
        )
        self.radar_params = radar_params
        self.mqtt_properties = mqtt_properties
        self._mqtt_connect(radar_params)

    def _mqtt_connect(self, radar_params):
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect(self.mqtt_properties['mqtt_host'], self.mqtt_properties['mqtt_port'], keepalive=30)
        payload = self.generate_json()
        self.mqtt_client.loop_start()





    def generate_json(self,lon = None,lat = None, target_id = 'radar', type_target_device = "device"):
        if type_target_device == 'target':
            stale =  (datetime.utcnow().replace(tzinfo=UTC) + timedelta(seconds=DEFAULT_TRACKING_TIMEOUT)).isoformat()
            start = datetime.utcnow().replace(tzinfo=UTC).isoformat()
            hae = 0
            status = None
        else:
            stale =  None
            start = None
            hae = self.radar_params.get('height',None)
            lat = self.radar_params['lat']
            lon = self.radar_params['lon']
            status = "OK"

        json_dict = {
            "version": "0.0.1",
            "source_uid": "magos-service",
            "time": datetime.utcnow().replace(tzinfo=UTC).isoformat(),
            "how": None,
            "status": status,
            "type": type_target_device,
        "stale": stale,
        "start": start,
        "uid": self.radar_params.get('uid',None),
        "description": None,
        "object_uid": target_id,
        "point": {
            "lat": lat,
            "lon": lon,
            "ce": None,
            "le": None,
            "hae": hae,
        },
        "spatial_orientation": None,
        "current_view_poly": None,
        "current_view_sensor_info": {"azimuth":self.radar_params['azimuth'],
                                     "elevation":self.radar_params['elevation'],
                                     "hfov":self.radar_params['hfov'],
                                     "vfov":self.radar_params['vfov'],
                                     "range":None},
        "camera_tracking": False,
        "alarmed": False
        }
        return json.dumps(json_dict, separators=(",", ":"))


    def publish_target(self,target_id,lat,lon, topic):

        payload = self.generate_json(lon,lat,target_id,type_target_device = 'target')
        self.mqtt_client.publish(
            topic,
            payload,
            qos=self.mqtt_properties.get('mqtt_qos', 0),
            retain=self.mqtt_properties.get('mqtt_retain', False),
        )

    def publish_radar(self, topic):
        self.mqtt_client.publish(
            topic,
            self.generate_json(),
            qos=self.mqtt_properties.get('mqtt_qos', 0),
        )




    def mqtt_publish(self: Dict[str, Any]):
        period = 1.0 / self._target_fps
        while True:
            item = self._ring.latest()  # direct read; no shared state
            if item is None: 
                self.publish_target(None,None,None,f"{self.mqtt_properties['mqtt_topic']}detections")
            else:
                [self.publish_target(target_id, lat, lon,f"{self.mqtt_properties['mqtt_topic']}detections") for target_id, lat, lon in zip(item['id'], item['lat'], item['lon'])]
                [self.publish_target(target_id, lat, lon,f"{self.mqtt_properties['mqtt_topic']}map/events") for target_id, lat, lon in zip(item['id'], item['lat'], item['lon'])]
                self.publish_radar(f"{self.mqtt_properties['mqtt_topic']}telemetry")
                self.publish_radar(f"{self.mqtt_properties['mqtt_topic']}map/events")
            time.sleep(period)

        
        
        
        # while True:
        #     item = self._ring.latest()  # direct read; no shared state
        #     payload = self._wrap_payload(item) if item is not None else {}

        #     if item is not None:
                 
        #         seq = payload.get("seq") 
        #         with self._lock:
        #             if seq is not None and seq == self._latest_seq:
        #                 # check of the detection is not new, if so, wait for a new one
        #                 time.sleep(self._period)
        #                 continue
        #             self._latest_seq = seq
                
        #         self.mqtt_client.publish(
        #             self.mqtt_properties['mqtt_topic'],
        #             json.dumps(payload, separators=(",", ":")),
        #             qos=self.mqtt_properties.get('mqtt_qos', 0),
        #             retain=self.mqtt_properties.get('mqtt_retain', False),
        #         )
        #     time.sleep(self._period)

               



    # def _sse_generator(self):
    #     # initial hello so client knows we're live
    #     yield "event: hello\ndata: {}\n\n"
    #     period = 1.0 / self._target_fps
    #     while True:
    #         item = self._ring.latest()  # direct read; no shared state
    #         payload = self._wrap_payload(item) if item is not None else {}
    #         yield f"data: {json.dumps(payload, separators=(',', ':'))}\n\n"
    #         time.sleep(period)




    # # ---------- routes ----------
    
    # def _routes(self):
    #     @self.app.get("/radar/stream")
    #     def radar_stream():
    #         headers = {
    #             "Cache-Control": "no-cache, no-transform",
    #             "Connection": "keep-alive",
    #             "X-Accel-Buffering": "no",
    #         }
    #         return StreamingResponse(self._sse_generator(),
    #                                  media_type="text/event-stream",
    #                                  headers=headers)


    #     @self.app.get("/radar/latest")
    #     def radar_latest():
    #         item = self._ring.latest()
    #         if item is None:
    #             return JSONResponse({"status": "empty"}, status_code=204)
    #         return JSONResponse(self._wrap_payload(item))

    # # ---------- server ----------
    # def start(self, host: str = "0.0.0.0", port: int = 9050):
    #     import uvicorn
    #     uvicorn.run(self.app, host=host, port=port, log_level="info")