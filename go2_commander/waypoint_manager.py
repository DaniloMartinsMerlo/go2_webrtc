# go2_commander/waypoint_manager.py
from pathlib import Path
import json


class WaypointManager:
    def __init__(self):
        self.waypoints = {}
        self.names = []

    def load(self, filename: str):
        p = Path(filename)
        if not p.exists():
            raise FileNotFoundError(f"Waypoints file not found: {filename}")

        with open(p, "r") as f:
            data = json.load(f)

        if not isinstance(data, dict):
            raise ValueError("Waypoints JSON root must be an object mapping names -> poses")

        # minimal validation
        for name, payload in data.items():
            if "position_x" not in payload or "position_y" not in payload or "orientation_w" not in payload:
                raise ValueError(f"Waypoint '{name}' missing required fields")

        self.waypoints = data
        self.names = list(data.keys())
        return self.names

    def get(self, name: str):
        return self.waypoints.get(name)

    def count(self):
        return len(self.names)
