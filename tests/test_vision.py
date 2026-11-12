import io
import math
from types import SimpleNamespace

import numpy as np
import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from sim.camera import R_BC, DeckCamera, rot_rpy
from sim.deck import DeckMotion
from vision.detector import PadDetector
from vision.landing_target import ardupilot_los_body, send_landing_target, target_angles
from vision.pad import Camera, Pad

D = math.radians


@pytest.fixture(scope="module")
def setup():
    cam = Camera.load("config/cam.json")
    pad = Pad.load("config/pad.json")
    return cam, pad, DeckCamera(cam, pad, seed=1), PadDetector(cam, pad)


def ap_target_ned(det, veh_pos, veh_rpy):
    """Where ArduPilot will think the pad is, decoding our angles exactly as it does."""
    ax, ay, dist = target_angles(det.tvec)
    los_ned = rot_rpy(*veh_rpy) @ ardupilot_los_body(ax, ay)
    return np.asarray(veh_pos) + los_ned * dist


CASES = [
    # veh_pos (NED), veh_rpy, deck_pos, deck_rpy
    ([0, 0, -5], (0, 0, 0), [0.3, -0.2, 0], (0, 0, 0)),
    ([1, 2, -4], (D(10), D(-5), D(40)), [1.5, 1.6, 0], (D(8), D(-3), D(30))),
    ([0, 0, -8], (D(-4), D(6), D(-120)), [1.0, -1.5, -0.3], (D(-10), D(6), D(60))),
    ([6, 4, -1.0], (D(3), D(3), D(10)), [6.05, 3.95, 0], (D(5), 0, D(30))),
]


@pytest.mark.parametrize("veh_pos,veh_rpy,deck_pos,deck_rpy", CASES)
def test_round_trip_matches_ardupilot_decoding(setup, veh_pos, veh_rpy, deck_pos, deck_rpy):
    cam, pad, camera, detector = setup
    img = camera.render(veh_pos, veh_rpy, deck_pos, deck_rpy)
    det = detector.detect(img)
    assert det is not None, "pad not detected"
    est = ap_target_ned(det, veh_pos, veh_rpy)
    horiz_err = np.hypot(*(est[:2] - np.asarray(deck_pos[:2])))
    assert horiz_err < 0.03 + 0.005 * abs(veh_pos[2]), f"horizontal error {horiz_err:.3f} m"


def test_small_marker_takes_over_low(setup):
    cam, pad, camera, detector = setup
    det = detector.detect(camera.render([0, 0, -0.4], (0, 0, 0), [0, 0, 0], (0, 0, 0)))
    assert det is not None and det.ids == [1]
    assert abs(det.distance - 0.4) < 0.02


def test_big_marker_acquires_high(setup):
    cam, pad, camera, detector = setup
    det = detector.detect(camera.render([0, 0, -9], (0, 0, 0), [1, 1, 0], (0, 0, 0)))
    assert det is not None and 0 in det.ids


def test_sign_convention_target_ahead_and_right():
    # landing point ahead (image up = -y_cam) and right (+x_cam) of a level vehicle
    tvec = np.array([0.5, -1.0, 5.0])
    los = ardupilot_los_body(*target_angles(tvec)[:2])
    assert los[0] > 0 and los[1] > 0
    np.testing.assert_allclose(los, R_BC @ tvec / np.linalg.norm(tvec), atol=1e-9)


def test_landing_target_packs_and_decodes(setup):
    buf = io.BytesIO()
    conn = SimpleNamespace(mav=mavlink.MAVLink(buf, srcSystem=1, srcComponent=191))
    det = SimpleNamespace(tvec=np.array([0.2, -0.1, 3.0]))
    send_landing_target(conn, det, 123456)
    msg = mavlink.MAVLink(None).parse_buffer(buf.getvalue())[0]
    ax, ay, dist = target_angles(det.tvec)
    assert msg.get_type() == "LANDING_TARGET"
    assert msg.frame == mavlink.MAV_FRAME_BODY_FRD and msg.position_valid == 0
    assert msg.angle_x == pytest.approx(ax, abs=1e-6)
    assert msg.angle_y == pytest.approx(ay, abs=1e-6)
    assert msg.distance == pytest.approx(dist, abs=1e-5)


@pytest.mark.parametrize("name", ["calm", "moderate", "rough"])
def test_platform_reduces_tilt(name):
    path = f"sim/scenarios/{name}.yaml"
    raw, stab = DeckMotion.load(path), DeckMotion.load(path, stab_gain=0.85)
    ts = np.linspace(0, 60, 600)
    assert max(stab.tilt_deg(t) for t in ts) < 0.2 * max(raw.tilt_deg(t) for t in ts)
