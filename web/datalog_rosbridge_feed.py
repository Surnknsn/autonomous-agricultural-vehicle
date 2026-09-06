#!/usr/bin/env python3
import json
import math
import os
import time

import requests
import websocket


ROSBRIDGE_URL = os.getenv("DATALOG_ROSBRIDGE_URL", "ws://127.0.0.1:9090")
DATALOG_API_URL = os.getenv("DATALOG_API_URL", "http://127.0.0.1:8080/api/datalog/current_gps")
THROTTLE_MS = int(os.getenv("DATALOG_GPS_FEED_THROTTLE_MS", "200"))
DEBUG_LOG = os.getenv("DATALOG_GPS_FEED_DEBUG", "0").lower() in ("1", "true", "yes", "on")


def clean_json_data(data):
    out = []
    for value in data or []:
        try:
            f = float(value)
            out.append(f if math.isfinite(f) else None)
        except Exception:
            out.append(None)
    return out


def main():
    print(f"datalog feed starting rosbridge={ROSBRIDGE_URL} api={DATALOG_API_URL}", flush=True)
    while True:
        ws = None
        try:
            ws = websocket.create_connection(ROSBRIDGE_URL, timeout=5)
            print("datalog feed connected to rosbridge", flush=True)
            ws.send(json.dumps({
                "op": "subscribe",
                "topic": "/current_gps",
                "type": "std_msgs/Float32MultiArray",
                "throttle_rate": THROTTLE_MS,
            }))
            while True:
                payload = json.loads(ws.recv())
                if payload.get("op") != "publish" or payload.get("topic") != "/current_gps":
                    continue
                data = (payload.get("msg") or {}).get("data", [])
                if len(data) < 4:
                    continue
                try:
                    res = requests.post(DATALOG_API_URL, json={"data": clean_json_data(data)}, timeout=2)
                    if DEBUG_LOG:
                        print(f"posted gps len={len(data)} status={res.status_code}", flush=True)
                except Exception as exc:
                    print(f"post failed: {exc}", flush=True)
        except Exception as exc:
            print(f"feed reconnect after error: {exc}", flush=True)
            time.sleep(2.0)
        finally:
            try:
                if ws is not None:
                    ws.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
