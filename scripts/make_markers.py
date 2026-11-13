"""Printable ArUco markers and the full landing pad, from config/pad.json.

    python -m scripts.make_markers          # writes markers/

Each image is labelled with the size its black square must measure when printed. Distances from
the detector are only correct if the printed size matches pad.json.
"""
import argparse
import os

import cv2

from sim.camera import build_pad_texture
from vision.pad import Pad


def label(img, text):
    pad = 70
    out = cv2.copyMakeBorder(img, 0, pad, 0, 0, cv2.BORDER_CONSTANT, value=255)
    cv2.putText(out, text, (20, img.shape[0] + 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pad", default="config/pad.json")
    p.add_argument("--out", default="markers")
    args = p.parse_args()

    pad = Pad.load(args.pad)
    os.makedirs(args.out, exist_ok=True)
    adict = pad.aruco_dict()
    written = []

    for m in pad.markers:
        img = cv2.aruco.generateImageMarker(adict, m.id, 800, borderBits=1)
        img = cv2.copyMakeBorder(img, 160, 160, 160, 160, cv2.BORDER_CONSTANT, value=255)
        cm = m.size * 100
        path = os.path.join(args.out, f"marker{m.id}_{cm:g}cm.png")
        cv2.imwrite(path, label(img, f"ArUco {pad.dictionary} id {m.id}: print so black square = {cm:g} cm"))
        written.append(path)

    tex = build_pad_texture(pad, ppm=600)[0][0]
    path = os.path.join(args.out, f"pad_{pad.size:g}m.png")
    cv2.imwrite(path, label(tex, f"Landing pad: print {pad.size * 100:g} x {pad.size * 100:g} cm; "
                                 f"landing point = centre of id 1; top edge = forward"))
    written.append(path)

    for w in written:
        print(w)


if __name__ == "__main__":
    main()
