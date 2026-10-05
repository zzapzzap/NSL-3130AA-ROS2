#!/usr/bin/env python3
"""Generate an rviz2 ``QMainWindow State`` hex blob for the multiview viewer.

Layout: a narrow **Displays** column on the left, each camera's **cam_<id> RGB**
image stacked on the right, and **Time** across the bottom. The central 3D
orbit view keeps the remaining space. multiview.launch.py
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

    win.setCorner(Qt.BottomLeftCorner, Qt.BottomDockWidgetArea)
    win.setCorner(Qt.BottomRightCorner, Qt.BottomDockWidgetArea)
    win.addDockWidget(Qt.LeftDockWidgetArea, docks['Displays'])
    images = [docks[f'cam_{c} RGB'] for c in cam_ids]
    if images:
        win.addDockWidget(Qt.RightDockWidgetArea, images[0])
        for previous, current in zip(images, images[1:]):
            win.splitDockWidget(previous, current, Qt.Vertical)
    # Time has a wide minimum size in RViz; keeping it out of the left column
    # prevents it from forcing Displays to grow to the same width.
    win.addDockWidget(Qt.BottomDockWidgetArea, docks['Time'])
    win.show()
    app.processEvents()
    win.resizeDocks([docks['Displays']], [380], Qt.Horizontal)
    if images:
        win.resizeDocks([images[0]], [360], Qt.Horizontal)
        win.resizeDocks(images, [450] * len(images), Qt.Vertical)
    win.resizeDocks([docks['Time']], [60], Qt.Vertical)
    app.processEvents()
    state = bytes(win.saveState()).hex()
    win.close()
    return state


if __name__ == '__main__':
    ids = [a.strip() for a in sys.argv[1:] if a.strip()]
    try:
        sys.stdout.write(build_state(ids))
    except Exception as e:  # noqa: BLE001 — any failure → launch falls back
        sys.stderr.write(f'gen_multiview_rviz_layout: {e}\n')
        sys.exit(1)
