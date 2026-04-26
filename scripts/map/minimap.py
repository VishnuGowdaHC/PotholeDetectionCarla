import json
import threading
import queue

data_queue = queue.Queue()
POTHOLE_FILE = "potholes.json"

def extractBaseMap(world):
    waypoints  = world.get_map().generate_waypoints(8.0)
    road_segments = []
    for wp in waypoints:
        next_wps = wp.next(8.0)
        for next_wp in next_wps:
            p1 = {'x': wp.transform.location.x, 'y': -wp.transform.location.y}
            p2 = {'x': next_wp.transform.location.x, 'y': -next_wp.transform.location.y}
            road_segments.append({'start': p1, 'end': p2})
    print(f"Map segments: {len(road_segments)}, sample: {road_segments[0]}")
    return road_segments

with open(POTHOLE_FILE, "w") as f:
    json.dump({"map": [], "hits": []}, f)

def fileWriter():
    while True:
        payload = data_queue.get()
        try:
            with open(POTHOLE_FILE, "r+") as f:
                data = json.load(f)
        except:
            data = {"map": [], "hits": []}

        if payload.get("type") == "init_map":
            data["map"] = payload["data"]
        else:
            data["hits"].append(payload)

        with open(POTHOLE_FILE, "w") as f:
            json.dump(data, f)

writer_thread = threading.Thread(target=fileWriter, daemon=True)
writer_thread.start()