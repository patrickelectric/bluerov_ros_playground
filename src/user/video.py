import cv2
import numpy as np
import threading


class Video(object):
    def __init__(self, ip='127.0.0.1', port=5600):
        self.ip = ip
        self.port = port
        self._frame = None

    def start_capture(self):
        'Start video capture'
        self.cap = cv2.VideoCapture('udp://{0}:{1}'.format(self.ip, self.port))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame(True)

    def frame(self, fake=False):
        'Return frame'
        if fake:
            # Update frame buffer
            self._frame = self.cap.read()
            thread = threading.Timer(1 / self.fps, self.frame, [True])
            thread.daemon = True
            thread.start()
        return self._frame

    def __exit__(self, exc_type, exc_value, traceback):
        'Release camera'
        self.cap.release()


if __name__ == '__main__':
    vid = Video()
    vid.start_capture()

    while(True):
        ret, frame = vid.frame()
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
