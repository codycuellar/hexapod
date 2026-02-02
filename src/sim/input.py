"""
Simple 2D gamepad input visualization.

Shows left/right joystick positions as dots, trigger levels as side bars,
and on/off indicators for bumpers, face buttons (A/B/X/Y), L3/R3, and D-pad.
Run with: python -m sim.input
"""

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches

from hexapod.gamepad import get_controller

# Button names and display labels (use button_held)
BUTTONS = [
    ("bumper_l", "L1", "tab:blue"),
    ("a", "A", "tab:green"),
    ("b", "B", "tab:red"),
    ("x", "X", "tab:purple"),
    ("y", "Y", "tab:olive"),
    ("bumper_r", "R1", "tab:orange"),
    ("l3", "L3", "tab:cyan"),
    ("r3", "R3", "tab:pink"),
]

# D-pad: (label, color, getter) - dpad uses live values
DPAD_THRESHOLD = 0.3
DPAD_BUTTONS = [
    ("U", "tab:gray", lambda g: g.dpad.y > DPAD_THRESHOLD),
    ("D", "tab:gray", lambda g: g.dpad.y < -DPAD_THRESHOLD),
    ("L", "tab:gray", lambda g: g.dpad.x < -DPAD_THRESHOLD),
    ("R", "tab:gray", lambda g: g.dpad.x > DPAD_THRESHOLD),
]


def main() -> None:
    gamepad = get_controller()
    gamepad.start_reading()

    plt.ion()
    fig = plt.figure(figsize=(8, 6), layout="constrained")
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
        rect = mpatches.FancyBboxPatch(
            (i + 0.1, -0.35), 0.8, 0.7,
            boxstyle="round,pad=0.02", linewidth=1.5,
            edgecolor=color, facecolor="white",
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
    for i, (label, color, _) in enumerate(DPAD_BUTTONS):
        rect = mpatches.FancyBboxPatch(
            (i + 0.1, -0.35), 0.8, 0.7,
            boxstyle="round,pad=0.02", linewidth=1.5,
            edgecolor=color, facecolor="white",
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
            dot_l.set_data([gamepad.joy_l.x], [gamepad.joy_l.y])
            dot_r.set_data([gamepad.joy_r.x], [gamepad.joy_r.y])
            bar_l.set_height(gamepad.trigger_l)
            bar_r.set_height(gamepad.trigger_r)
            for (rect, color), (name, _, _) in zip(patches, BUTTONS):
                rect.set_facecolor(color if gamepad.button_held(name) else "white")
            for (rect, color), (_, _, getter) in zip(dpad_patches, DPAD_BUTTONS):
                rect.set_facecolor(color if getter(gamepad) else "white")
            fig.canvas.draw_idle()
            plt.pause(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        gamepad.stop_reading()
        plt.ioff()
        plt.close()


if __name__ == "__main__":
    main()
