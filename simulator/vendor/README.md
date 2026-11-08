# Files from other projects

- `mediapipe/`: `@mediapipe/tasks-vision` 1.1.1 from npm, Apache 2.0, by Google.
- `models/gesture_recognizer.task` and `models/efficientdet_lite0.tflite`: from Google's MediaPipe model storage, Apache 2.0.

They are unchanged. `python tools/get_camera_files.py` downloads `mediapipe/` and `models/`, which are too big for the repository; after that the page needs no internet.
- `testdata/`: a few pictures for the tests: `hand_*` from MediaPipe's gesture test data, `apple.jpg` and `basketball1.png` from OpenCV's samples (both Apache 2.0).
