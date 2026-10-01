"""
ROBOARM V5 — any serial arm, one robot file.

    python3 -m roboarm_v5                       # last robot used (or the default)
    python3 -m roboarm_v5 --robot robots/x.json
    python3 -m roboarm_v5 --sim --windowed      # no hardware: simulated board

Esc quits.
"""

import argparse
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROBOT = os.path.join(os.path.dirname(HERE), "robots", "roboarm_opi.json")


def parse(argv):
    p = argparse.ArgumentParser(prog="roboarm_v5", description=__doc__.split("\n")[1])
    p.add_argument("--robot", help="robot file (JSON); default: last used")
    p.add_argument("--sim", action="store_true",
                   help="simulated GPIO — run anywhere, no arm attached")
    p.add_argument("--windowed", action="store_true", help="don't go fullscreen")
    p.add_argument("--momentary-estop", action="store_true",
                   help="E-STOP clears on release instead of latching (V3 behaviour)")
    return p.parse_args(argv)


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)

    import tkinter as tk
    from .log import Log
    from .io.config import Config
    from .robot.loader import load_robot, RobotFileError
    from .runtime import Runtime
    from .ui.theme import Type, Metrics
    from .ui.app import RoboArm

    log = Log(echo=args.sim)
    config = Config(log).load()
    path = args.robot or config["robot"] or DEFAULT_ROBOT
    try:
        model = load_robot(path)
    except (OSError, RobotFileError) as e:
        print("roboarm_v5: cannot load robot file: {}".format(e), file=sys.stderr)
        return 2
    config["robot"] = model.source

    rt = Runtime(model, sim=args.sim, estop_latching=not args.momentary_estop,
                 log=log, config=config).start()

    def switch_robot(new_path):
        config["robot"] = os.path.abspath(new_path)
        rt.save_config()
        rt.shutdown()
        root.destroy()
        argv2 = [a for a in sys.argv[1:]]
        if "--robot" in argv2:
            i = argv2.index("--robot")
            del argv2[i:i + 2]
        os.chdir(os.path.dirname(HERE))      # so "-m roboarm_v5" resolves
        os.execv(sys.executable, [sys.executable, "-m", "roboarm_v5",
                                  "--robot", config["robot"]] + argv2)

    root = tk.Tk()
    Type.resolve(root)
    Metrics.resolve(root)
    app = RoboArm(root, rt, on_switch_robot=switch_robot, fullscreen=not args.windowed)
    root.protocol("WM_DELETE_WINDOW", app.quit_app)
    # kill / systemctl stop / Ctrl-C: the same orderly shutdown as EXIT —
    # motors stopped, fan off, GPIO released. The handler runs on the next
    # Tk tick (every 100 ms), in the main thread.
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: root.after(0, app.quit_app))
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.quit_app()
    return 0


if __name__ == "__main__":
    sys.exit(main())
