"""Ship-deck motion from a sea-state scenario, with an optional 2-DOF stabilised platform.

Motion on each axis is a sum of sinusoids [amplitude, period_s, phase_deg]; roll/pitch amplitudes
are in degrees, heave/surge/sway in metres. The platform cancels a fraction `stab_gain` of deck
roll and pitch (it cannot remove heave or translation).

SITL limitation: the simulated vehicle still touches down on flat ground at NED z=0, so heave and
tilt only affect what the camera sees (and the touchdown score), not the contact physics.
"""
import json
import math

import numpy as np


def _sines(terms, t):
    return sum(a * math.sin(2 * math.pi * t / p + math.radians(ph)) for a, p, ph in terms or [])


class DeckMotion:
    def __init__(self, scenario, stab_gain=0.0):
        self.s = scenario
        self.stab_gain = stab_gain
        d = scenario["deck"]
        self.n0, self.e0 = d["north"], d["east"]
        self.heading = math.radians(d.get("heading_deg", 0.0))

    @classmethod
    def load(cls, path, stab_gain=0.0):
        with open(path) as f:
            return cls(json.load(f), stab_gain)

    @property
    def name(self):
        return self.s.get("name", "unnamed")

    def pose(self, t):
        """Pad pose at time t (s): (pos NED (3,), (roll, pitch, yaw) rad)."""
        m = self.s.get("motion", {})
        drift_n, drift_e = m.get("drift_ne", [0.0, 0.0])
        surge, sway = _sines(m.get("surge"), t), _sines(m.get("sway"), t)
        ch, sh = math.cos(self.heading), math.sin(self.heading)
        n = self.n0 + drift_n * t + ch * surge - sh * sway
        e = self.e0 + drift_e * t + sh * surge + ch * sway
        d = -_sines(m.get("heave"), t)
        k = 1.0 - self.stab_gain
        roll = k * math.radians(_sines(m.get("roll"), t))
        pitch = k * math.radians(_sines(m.get("pitch"), t))
        return np.array([n, e, d]), (roll, pitch, self.heading)

    def tilt_deg(self, t):
        _, (r, p, _) = self.pose(t)
        return math.degrees(math.acos(math.cos(r) * math.cos(p)))
