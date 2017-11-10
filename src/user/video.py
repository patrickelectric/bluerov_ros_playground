#!/usr/bin/env python
"""
BlueRov video capture class
"""
import threading

import cv2

class Video(threading.Thread):

    """BlueRov video capture class constructor

    Attributes:
        cap (VideoCapture): Camera capture object
        ip (string): Video udp ip address
        port (int): Video udp port
    """

    def __init__(self, ip='127.0.0.1', port=5600):
        super(Video, self).__init__()

        self.ip = ip
        self.port = port
        self._frame = None

        self.cap = cv2.VideoCapture('udp://{0}:{1}'.format(self.ip, self.port))

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return self._frame != None

    def run(self):
        """ Get frame to update _frame
        """
        while True:
            try:
                new_frame = self.cap.read()
                if new_frame != None:
                    self._frame = new_frame

            except Exception as error:
                print(error)

    def __exit__(self, exc_type, exc_value, traceback):
        'Release camera'
        self.cap.release()


if __name__ == '__main__':
    video = Video()
    video.start()

    while not video.frame_available():
        pass

    while True:
        ret, frame = video.frame()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
