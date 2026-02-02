"""
Simple 2D gamepad input visualization.

Shows left/right joystick positions as dots, trigger levels as side bars,
and on/off indicators for bumpers, face buttons (A/B/X/Y), L3/R3, and D-pad.
Run with: python -m sim.input

Set SIM_INPUT_FPS (default 15) for update rate; lower = less CPU (e.g. 10 on Pi).
"""

import os

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches

from hexapod.gamepad import get_controller

# (getter returns held for display, label, color)
BUTTONS = [
    (lambda g: g.get_bumper_l()[1], "BL", "tab:blue"),
    (lambda g: g.get_btn_south()[1], "S", "tab:green"),
    (lambda g: g.get_btn_east()[1], "E", "tab:red"),
    (lambda g: g.get_btn_west()[1], "W", "tab:purple"),
    (lambda g: g.get_btn_north()[1], "N", "tab:olive"),
    (lambda g: g.get_bumper_r()[1], "BR", "tab:orange"),
    (lambda g: g.get_joy_l_click()[1], "JL", "tab:cyan"),
    (lambda g: g.get_joy_r_click()[1], "JR", "tab:pink"),
]

# D-pad: (getter, label, color)
DPAD_BUTTONS = [
    (lambda g: g.get_dpad_up()[1], "DU", "tab:gray"),
    (lambda g: g.get_dpad_down()[1], "DD", "tab:gray"),
    (lambda g: g.get_dpad_left()[1], "DL", "tab:gray"),
    (lambda g: g.get_dpad_right()[1], "DR", "tab:gray"),
]


def main() -> None:
    gamepad = get_controller()
    gamepad.start_reading()

    try:
        fps = float(os.environ.get("SIM_INPUT_FPS", "15"))
    except ValueError:
        fps = 15.0
    interval_s = 1.0 / max(fps, 5.0)  # clamp min 5 fps

    plt.ion()
    fig = plt.figure(figsize=(6, 4.5), layout="constrained")
    gs = gridspec.GridSpec(
        3, 3,
        figure=fig,
        width_ratios=[0.15, 1, 0.15],
        height_ratios=[1, 0.3, 0.25],
        wspace=0.3,
        hspace=0.35,
    )

    # Main joystick area: -1 to 1
    ax_main = fig.add_subplot(gs[0, 1])
    ax_main.set_xlim(-1, 1)
    ax_main.set_ylim(-1, 1)
    ax_main.set_aspect("equal")
    ax_main.axhline(0, color="gray", linewidth=0.5)
    ax_main.axvline(0, color="gray", linewidth=0.5)
    ax_main.set_xlabel("x")
    ax_main.set_ylabel("y")
    ax_main.set_title("Joysticks")

    dot_l, = ax_main.plot([], [], "o", color="tab:blue", markersize=14, label="L")
    dot_r, = ax_main.plot([], [], "o", color="tab:orange", markersize=14, label="R")
    ax_main.legend(loc="upper right")

    # Left trigger bar (vertical, 0-1)
    ax_lt = fig.add_subplot(gs[0, 0])
    ax_lt.set_xlim(-0.5, 0.5)
    ax_lt.set_ylim(0, 1)
    ax_lt.set_ylabel("L trigger")
    ax_lt.set_xticks([])
    bar_l = ax_lt.bar(0, 0, width=0.3, color="tab:blue", align="center")[0]

    # Right trigger bar (vertical, 0-1)
    ax_rt = fig.add_subplot(gs[0, 2])
    ax_rt.set_xlim(-0.5, 0.5)
    ax_rt.set_ylim(0, 1)
    ax_rt.set_ylabel("R trigger")
    ax_rt.set_xticks([])
    ax_rt.yaxis.tick_right()
    bar_r = ax_rt.bar(0, 0, width=0.3, color="tab:orange", align="center")[0]

    # Button indicators: L1, A, B, X, Y, R1, L3, R3
    ax_btns = fig.add_subplot(gs[1, 1])
    ax_btns.set_xlim(-0.5, 7.5)
    ax_btns.set_ylim(-0.5, 0.5)
    ax_btns.axis("off")
    patches = []
    for i, (name, label, color) in enumerate(BUTTONS):
        rect = mpatches.Rectangle(
            (i + 0.1, -0.35), 0.8, 0.7,
            linewidth=1.5, edgecolor=color, facecolor="white",
            clip_on=False,
        )
        ax_btns.add_patch(rect)
        txt = ax_btns.text(i + 0.5, 0, label, ha="center", va="center", fontsize=9)
        txt.set_clip_on(False)
        patches.append((rect, color))

    # D-pad indicators: U, D, L, R
    ax_dpad = fig.add_subplot(gs[2, 1])
    ax_dpad.set_xlim(-0.5, 3.5)
    ax_dpad.set_ylim(-0.5, 0.5)
    ax_dpad.axis("off")
    dpad_patches = []
    for i, (getter, label, color) in enumerate(DPAD_BUTTONS):
        rect = mpatches.Rectangle(
            (i + 0.1, -0.35), 0.8, 0.7,
            linewidth=1.5, edgecolor=color, facecolor="white",
            clip_on=False,
        )
        ax_dpad.add_patch(rect)
        txt = ax_dpad.text(i + 0.5, 0, label, ha="center", va="center", fontsize=9)
        txt.set_clip_on(False)
        dpad_patches.append((rect, color))

    fig.suptitle("Gamepad Input")
    plt.show()

    try:
        while plt.fignum_exists(fig.number):
            joy_l = gamepad.get_joy_l()
            joy_r = gamepad.get_joy_r()
            dot_l.set_data([joy_l.x], [joy_l.y])
            dot_r.set_data([joy_r.x], [joy_r.y])
            bar_l.set_height(gamepad.get_trigger_l())
            bar_r.set_height(gamepad.get_trigger_r())
            for (rect, color), (getter, _, _) in zip(patches, BUTTONS):
                rect.set_facecolor(color if getter(gamepad) else "white")
            for (rect, color), (getter, _, _) in zip(dpad_patches, DPAD_BUTTONS):
                rect.set_facecolor(color if getter(gamepad) else "white")
            fig.canvas.draw_idle()
            plt.pause(interval_s)
    except KeyboardInterrupt:
        pass
    finally:
        gamepad.stop_reading()
        plt.ioff()
        plt.close()


if __name__ == "__main__":
    main()
