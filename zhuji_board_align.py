#!/usr/bin/env python3
"""Run the imported alignment tools with zhuji's OpenCV 4.2.0."""
import sys

import cv2

import board_align


class FoxyCornerDetector:
    """Convert the old detector's bottom-first rows to the printed board axes."""

    def __init__(self, observer):
        self.board = observer.board
        self.dictionary = observer.dictionary
        self.intrinsics = observer.c
        self.columns = observer.nx - 1
        self.rows = observer.ny - 1

    def detectBoard(self, gray):
        markers, marker_ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        if marker_ids is None:
            return None, None, markers, marker_ids
        _, corners, ids = cv2.aruco.interpolateCornersCharuco(
            markers, marker_ids, gray, self.board,
            cameraMatrix=self.intrinsics['K'], distCoeffs=self.intrinsics['D'],
        )
        if ids is not None:
            ids = (self.rows - 1 - ids // self.columns) * self.columns + ids % self.columns
        return corners, ids, markers, marker_ids


class FoxyObserver(board_align.Observer):
    def __init__(self, board_config, intrinsics, *args, **kwargs):
        super().__init__(board_config, intrinsics, *args, **kwargs)
        if intrinsics is not None:
            if board_align.fingerprint(self.config) != '437d28755b5102513fd2204b356c468f7d9618172b30ba59d7d60f049d63d928':
                raise ValueError('OpenCV 4.2.0 adapter was verified only for the imported 14x9 board')
            self.detector = FoxyCornerDetector(self)


def main():
    if cv2.__version__ == '4.2.0':
        board_align.Observer = FoxyObserver
    try:
        if len(sys.argv) > 1 and sys.argv[1] == 'ros':
            del sys.argv[1]
            from observe_ros2 import main as observe
            observe()
        else:
            board_align.main()
    except (RuntimeError, ValueError) as error:
        raise SystemExit(str(error))


if __name__ == '__main__':
    main()
