"""
Unified gamepad interface for hexapod control.

Use get_controller() to obtain a controller instance (Xbox or DS4).
Read state via get_joy_*(), get_trigger_*(), get_bumper_*(), etc.
"""

import os
import sys

from hexapod.gamepad.base import Gamepad
from hexapod.gamepad.xbox import XboxController
from hexapod.gamepad.ds4 import DS4Controller


def _default_controller() -> str:
    """DS4 on Linux/Pi, Xbox on Windows."""
    if sys.platform == "win32":
        return "xbox"
    return "ds4"


def get_controller(name: str | None = None, **kwargs: object) -> Gamepad:
    """
    Return a gamepad controller instance.

    Args:
        name: "xbox" or "ds4". If None, uses GAMEPAD env var or platform default
              (DS4 on Linux/Pi, Xbox on Windows).
        **kwargs: Passed to controller constructor (e.g. interface="/dev/input/js0" for DS4).
    """
    if name is None:
        name = os.environ.get("GAMEPAD", _default_controller())
    name = name.lower()
    if name == "xbox":
        return XboxController(**kwargs)
    if name == "ds4":
        return DS4Controller(**kwargs)
    raise ValueError(f"Unknown controller: {name}")


__all__ = [
    "Gamepad",
    "XboxController",
    "DS4Controller",
    "get_controller",
]
