from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class Landmark:
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class HandState:
    gesture: str
    wrist_x: float
    wrist_y: float
    score: float = 1.0


def classify_landmarks(landmarks: Sequence[Landmark]) -> str:
    if len(landmarks) < 21:
        return "Unknown"

    thumb_open = _thumb_extended(landmarks)
    index_open = _finger_extended(landmarks, 5, 6, 8)
    middle_open = _finger_extended(landmarks, 9, 10, 12)
    ring_open = _finger_extended(landmarks, 13, 14, 16)
    pinky_open = _finger_extended(landmarks, 17, 18, 20)

    open_fingers = [thumb_open, index_open, middle_open, ring_open, pinky_open]
    open_count = sum(1 for open_finger in open_fingers if open_finger)

    if open_count >= 4:
        return "Open_Palm"
    if open_count == 0:
        return "Closed_Fist"
    if thumb_open and open_count == 1:
        return "Thumb_Up"
    if index_open and middle_open and not ring_open and not pinky_open:
        return "Victory"
    if index_open and open_count == 1:
        return "Pointing_Up"
    return "Unknown"


def _finger_extended(
    landmarks: Sequence[Landmark],
    mcp_index: int,
    pip_index: int,
    tip_index: int,
) -> bool:
    wrist = landmarks[0]
    mcp = landmarks[mcp_index]
    pip = landmarks[pip_index]
    tip = landmarks[tip_index]

    wrist_to_mcp = _distance(wrist, mcp)
    wrist_to_tip = _distance(wrist, tip)
    pip_to_tip = _distance(pip, tip)
    return wrist_to_tip > wrist_to_mcp * 1.18 and pip_to_tip > wrist_to_mcp * 0.35


def _thumb_extended(landmarks: Sequence[Landmark]) -> bool:
    wrist = landmarks[0]
    index_mcp = landmarks[5]
    thumb_tip = landmarks[4]
    thumb_ip = landmarks[3]

    palm_width = max(_distance(wrist, index_mcp), 1e-6)
    wrist_to_tip = _distance(wrist, thumb_tip)
    ip_to_tip = _distance(thumb_ip, thumb_tip)
    return wrist_to_tip > palm_width * 0.9 and ip_to_tip > palm_width * 0.25


def _distance(first: Landmark, second: Landmark) -> float:
    return math.sqrt(
        (first.x - second.x) * (first.x - second.x)
        + (first.y - second.y) * (first.y - second.y)
        + (first.z - second.z) * (first.z - second.z)
    )
