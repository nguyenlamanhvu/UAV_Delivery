import requests
import time

while True:
    try:
        requests.post("http://localhost:8082/command", json={"yaw": -1.0})
    except:
        pass
    time.sleep(0.05)
