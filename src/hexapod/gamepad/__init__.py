"""
Gamepad: common interface and controller implementations.

Use get_controller(name) to obtain the right controller for your platform or config.
"""

import os

from hexapod.gamepad.base import Gamepad
from hexapod.gamepad.ds4 import DS4Controller
from hexapod.gamepad.xbox import XboxController

# Map name -> class for get_controller()
_CONTROLLERS: dict[str, type[Gamepad]] = {
    "xbox": XboxController,
    "ds4": DS4Controller,
}


def get_controller(name: str | None = None, **kwargs: object) -> Gamepad:
    """
    Return a gamepad instance for the given name.

    Args:
        name: One of "xbox", "ds4", or None.
              If None, uses env var GAMEPAD if set, otherwise "xbox".
        **kwargs: Passed to the controller constructor (e.g. interface="/dev/input/js1" for DS4).

    Returns:
        A Gamepad subclass instance (not yet started; call start_reading()).
    """
    key = (name or os.environ.get("GAMEPAD", "xbox")).strip().lower()
    if key not in _CONTROLLERS:
        raise ValueError(
            f"Unknown gamepad '{key}'. Known: {', '.join(_CONTROLLERS)}"
        )
    return _CONTROLLERS[key](**kwargs)


__all__ = [
    "DS4Controller",
    "Gamepad",
    "XboxController",
    "get_controller",
]
