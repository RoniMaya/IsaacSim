# pip install paho-mqtt
import json, time, threading
import paho.mqtt.client as mqtt

class RadarMQTTPublisher:
    """
    Publishes detections from the same ring buffer your SSE code uses.
    - 'radar/stream' : live feed (QoS 0, non-retained)
    - 'radar/latest' : snapshot (QoS 1, retained)
    """
    def __init__(self,
                 ring_buffer,
                 broker_host="127.0.0.1",
                 broker_port=1883,
                 client_id="radar-publisher",
                 keepalive=30,
                 topic_stream="radar/stream",
                 topic_latest="radar/latest",
                 target_fps=1):
        self._ring = ring_buffer
        self._target_fps = target_fps
        self._topic_stream = topic_stream
        self._topic_latest = topic_latest
        self._stop = threading.Event()
        self._last_seq = None  # avoid re-publishing identical sample

        self._cli = mqtt.Client(client_id=client_id, clean_session=True)
        # If you use auth/TLS:
        # self._cli.username_pw_set("user", "pass")
        # self._cli.tls_set(ca_certs="ca.crt")  # etc.

        # reconnect backoff
        self._cli.reconnect_delay_set(min_delay=1, max_delay=10)
        self._cli.connect(broker_host, broker_port, keepalive)
        self._cli.loop_start()

        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def _wrap_payload(self, item):
        # identical behavior to your SSE wrapper
        now = time.time()
        if isinstance(item, dict):
            payload = dict(item)
            payload.setdefault("ts", now)
            payload.setdefault("seq", int(time.time_ns()))
            return payload
        return {"seq": int(time.time_ns()), "ts": now, "data": item}

    def _run(self):
        period = 1.0 / max(1, int(self._target_fps))
        while not self._stop.is_set():
            item = self._ring.latest()
            if item is not None:
                payload = self._wrap_payload(item)
                seq = payload.get("seq")
                # publish only when there is a new sample OR at the desired rate
                if seq != self._last_seq:
                    msg = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)

                    # live stream (low-latency, drops allowed)
                    self._cli.publish(self._topic_stream, msg, qos=0, retain=False)

                    # latest snapshot (new subscribers get it immediately)
                    self._cli.publish(self._topic_latest, msg, qos=1, retain=True)

                    self._last_seq = seq
            time.sleep(period)

    def stop(self):
        self._stop.set()
        self._thr.join(timeout=1.0)
        self._cli.loop_stop()
        self._cli.disconnect()
