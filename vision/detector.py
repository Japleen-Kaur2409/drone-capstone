"""ArUco landing-pad detector: all visible pad markers -> one PnP solve for the landing point."""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    ids: list
    rvec: np.ndarray        # deck -> camera rotation (Rodrigues)
    tvec: np.ndarray        # landing point in camera frame, m
    corners: list           # raw ArUco corners, for drawing
    reproj_px: float

    @property
    def distance(self):
        return float(np.linalg.norm(self.tvec))


class PadDetector:
    def __init__(self, cam, pad, min_side_px=12):
        self.cam = cam
        self.pad = pad
        self.min_side_px = min_side_px
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(pad.aruco_dict(), params)

    def detect(self, gray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        obj, img, used, used_corners = [], [], [], []
        for c, i in zip(corners, ids.ravel()):
            m = self.pad.marker(int(i))
            if m is None:
                continue
            c = c.reshape(4, 2)
            if np.min(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)) < self.min_side_px:
                continue
            obj.append(self.pad.object_points(m))
            img.append(c)
            used.append(int(i))
            used_corners.append(c.reshape(1, 4, 2))
        if not obj:
            return None
        obj = np.concatenate(obj).astype(np.float64)
        img = np.concatenate(img).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.cam.K, self.cam.dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok or tvec[2, 0] <= 0:
            return None
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.cam.K, self.cam.dist)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
        return Detection(used, rvec.ravel(), tvec.ravel(), used_corners, err)
