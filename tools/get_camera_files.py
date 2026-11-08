"""get_camera_files.py - downloads the files the page's optional gestures and object detection need, into simulator/vendor.

    python tools/get_camera_files.py

About 42 MB, from npm and Google's model storage, both under the Apache 2.0 licence. Flying needs none of this. They are
not in the repository because of their size.
"""
import io
import pathlib
import tarfile
import urllib.request

VENDOR = pathlib.Path(__file__).resolve().parent.parent / "simulator" / "vendor"
PACKAGE = "https://registry.npmjs.org/@mediapipe/tasks-vision/-/tasks-vision-1.1.1.tgz"
MODELS = {
    "gesture_recognizer.task": "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task",
    "efficientdet_lite0.tflite": "https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float16/1/efficientdet_lite0.tflite",
}
WANTED = ("package/package.json", "package/vision_bundle.mjs", "package/wasm/")


def fetch(url):
    print("Downloading", url, flush=True)
    with urllib.request.urlopen(url, timeout=300) as r:
        return r.read()


def main():
    package = VENDOR / "mediapipe"
    with tarfile.open(fileobj=io.BytesIO(fetch(PACKAGE)), mode="r:gz") as tar:
        for member in tar.getmembers():
            if member.isfile() and member.name.startswith(WANTED):
                dst = package / member.name[len("package/"):]
                dst.parent.mkdir(parents=True, exist_ok=True)
                dst.write_bytes(tar.extractfile(member).read())
    models = VENDOR / "models"
    models.mkdir(parents=True, exist_ok=True)
    for name, url in MODELS.items():
        (models / name).write_bytes(fetch(url))
    print("Done: the page's Gestures and Objects boxes can now be used.")


main()
