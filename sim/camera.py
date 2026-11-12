"""Simulated downward-facing camera looking at a landing pad on a moving deck.

Each pixel's ray is intersected with the deck plane and the pad texture is sampled there, so
tilt, heave, perspective and partial views are all geometrically exact.

Frames: NED world, body FRD, camera (x right, y down, z optical axis). The camera points straight
down with image-up = vehicle nose.
"""
import cv2
import numpy as np

# body <- camera
R_BC = np.array([[0.0, -1.0, 0.0],
                 [1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0]])

SEA = 60
PAD = 210


def rot_rpy(roll, pitch, yaw):
    """ZYX Euler (rad) -> rotation matrix, body -> NED."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def build_pad_texture(pad, ppm=800, levels=6):
    """Pad image (texture-up = deck forward) as a mip pyramid. Returns (pyramid, ppm)."""
    block = 2 ** levels
    side = int(np.ceil(pad.size * ppm / block) * block)
    ppm = side / pad.size
    tex = np.full((side, side), PAD, np.uint8)
    cv2.rectangle(tex, (0, 0), (side - 1, side - 1), 120, max(2, side // 150))
    adict = pad.aruco_dict()
    for m in pad.markers:
        n = int(round(m.size * ppm))
        img = cv2.aruco.generateImageMarker(adict, m.id, n, borderBits=1)
        cu = side / 2 + m.y * ppm
        cv = side / 2 - m.x * ppm
        u0, v0 = int(round(cu - n / 2)), int(round(cv - n / 2))
        tex[v0:v0 + n, u0:u0 + n] = img
    pyr = [tex]
    for _ in range(levels):
        pyr.append(cv2.pyrDown(pyr[-1]))
    return pyr, ppm


class DeckCamera:
    def __init__(self, cam, pad, ppm=800, noise_std=3.0, blur_px=0, seed=0):
        self.cam = cam
        self.pyr, self.ppm = build_pad_texture(pad, ppm)
        self.noise_std = noise_std
        self.blur_px = blur_px
        self.rng = np.random.default_rng(seed)
        u, v = np.meshgrid(np.arange(cam.width, dtype=np.float64),
                           np.arange(cam.height, dtype=np.float64))
        pix = np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
        self.rays = np.linalg.inv(cam.K) @ pix  # camera frame, z = 1

    def render(self, veh_pos, veh_rpy, deck_pos, deck_rpy):
        """veh_pos/deck_pos: NED (m). veh_rpy/deck_rpy: roll, pitch, yaw (rad). Returns uint8 gray."""
        R_nc = rot_rpy(*veh_rpy) @ R_BC
        R_nd = rot_rpy(*deck_rpy)
        d = R_nc @ self.rays
        n = R_nd[:, 2]
        rel = np.asarray(deck_pos, float) - np.asarray(veh_pos, float)
        denom = n @ d
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (n @ rel) / denom
        valid = (denom > 1e-9) & (t > 0)

        # pick the mip level whose resolution best matches the image at the image centre
        c = self.rays.shape[1] // 2 + self.cam.width // 2
        rng_c = t[c] if valid[c] else np.linalg.norm(rel)
        img_ppm = self.cam.K[0, 0] / max(rng_c, 1e-3)
        k = int(np.clip(np.floor(np.log2(self.ppm / img_ppm)), 0, len(self.pyr) - 1))
        tex, ppm_k = self.pyr[k], self.ppm / 2 ** k

        tt = np.where(valid, t, 0.0)
        hit = d * tt - rel[:, None]          # NED offset from deck centre
        local = R_nd.T @ hit                 # deck frame
        side = tex.shape[1]
        map_x = side / 2 + local[1] * ppm_k - 0.5
        map_y = side / 2 - local[0] * ppm_k - 0.5
        map_x[~valid] = -1e4
        map_y[~valid] = -1e4
        shape = (self.cam.height, self.cam.width)
        img = cv2.remap(tex, map_x.reshape(shape).astype(np.float32),
                        map_y.reshape(shape).astype(np.float32),
                        cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=SEA)

        if self.blur_px > 0:
            k = 2 * int(self.blur_px) + 1
            img = cv2.GaussianBlur(img, (k, k), 0)
        if self.noise_std > 0:
            noise = self.rng.normal(0, self.noise_std, img.shape)
            img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return img
