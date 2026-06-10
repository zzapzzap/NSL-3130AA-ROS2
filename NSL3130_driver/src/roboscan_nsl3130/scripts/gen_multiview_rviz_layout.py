#!/usr/bin/env python3
"""Generate an rviz2 ``QMainWindow State`` hex blob for the multiview viewer.

Layout: the **Displays** (topic/status) panel on top of the left column, each
camera's **cam_<id> RGB** image panel stacked below it, and **Time** at the
bottom — the 3D orbit view keeps the rest of the window. multiview.launch.py
injects the printed hex into the generated config's ``Window Geometry`` so the
arrangement survives a relaunch (rviz restores docks by objectName).

Usage:
    gen_multiview_rviz_layout.py 51 52 [...]   # prints the hex blob to stdout

On any failure (e.g. PyQt5 unavailable, no offscreen platform) it writes a
message to stderr and exits non-zero so the launch can omit the blob and fall
back to rviz's default docking.
"""

import os
import sys

# rviz dock layout is a serialized Qt state; build it headless.
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def build_state(cam_ids):
    from PyQt5.QtWidgets import QApplication, QMainWindow, QDockWidget, QWidget
    from PyQt5.QtCore import Qt

    app = QApplication(['gen-rviz-layout'])    # keep a reference (else GC'd)
    win = QMainWindow()
    win.resize(1850, 1016)
    win.setCentralWidget(QWidget())            # rviz's 3D render view sits here

    def dock(name):
        d = QDockWidget(name, win)
        d.setObjectName(name)                  # rviz matches docks by objectName
        d.setWidget(QWidget())
        return d

    names = ['Displays'] + [f'cam_{c} RGB' for c in cam_ids] + ['Time']
    docks = {n: dock(n) for n in names}

    # Single left column, top → bottom: Displays, each RGB image, Time.
    win.addDockWidget(Qt.LeftDockWidgetArea, docks['Displays'])
    prev = docks['Displays']
    for n in names[1:]:
        win.splitDockWidget(prev, docks[n], Qt.Vertical)
        prev = docks[n]

    # Bias the column: Displays tall, each image medium, Time short; ~420 px wide.
    heights = [360] + [300] * len(cam_ids) + [80]
    win.resizeDocks(list(docks.values()), heights, Qt.Vertical)
    win.resizeDocks([docks['Displays']], [420], Qt.Horizontal)

    return bytes(win.saveState()).hex()


if __name__ == '__main__':
    ids = [a.strip() for a in sys.argv[1:] if a.strip()]
    try:
        sys.stdout.write(build_state(ids))
    except Exception as e:  # noqa: BLE001 — any failure → launch falls back
        sys.stderr.write(f'gen_multiview_rviz_layout: {e}\n')
        sys.exit(1)
