"""Landing pad layout and camera intrinsics, shared by the detector and the simulator."""
import json
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Marker:
    id: int
    size: float  # outer black-square side, m
    x: float     # centre, m forward of landing point
    y: float     # centre, m right of landing point


@dataclass
class Pad:
    dictionary: str
    size: float
    markers: list

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(d["dictionary"], d["size"], [Marker(**m) for m in d["markers"]])

    def aruco_dict(self):
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))

    def marker(self, marker_id):
        return next((m for m in self.markers if m.id == marker_id), None)

    @staticmethod
    def object_points(m):
        """Marker corners in deck frame (x fwd, y right, z down), ArUco order TL, TR, BR, BL.

        Marker images are laid on the pad with image-up = deck forward.
        """
        h = m.size / 2
        return np.array([[m.x + h, m.y - h, 0.0],
                         [m.x + h, m.y + h, 0.0],
                         [m.x - h, m.y + h, 0.0],
                         [m.x - h, m.y - h, 0.0]])


@dataclass
class Camera:
    width: int
    height: int
    K: np.ndarray
    dist: np.ndarray

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(d["width"], d["height"],
                   np.array(d["K"], dtype=np.float64), np.array(d["dist"], dtype=np.float64))
