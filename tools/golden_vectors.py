"""Builds reference packets with the OFFICIAL simulator code (uav-hil-simulator arduino_hil/hil_protocol.py)
and stores them in tests/golden_vectors.json.  The simulator self-test checks that our JavaScript packet
builder produces byte-identical packets, and the firmware tests use the same framing.

    python tools/golden_vectors.py [path\\to\\uav-hil-simulator-main]
"""
import importlib.util
import json
import pathlib
import sys

root = pathlib.Path(__file__).resolve().parent.parent
default_sim = root.parent / "uav-hil-simulator-main" / "uav-hil-simulator-main"
sim = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else default_sim
spec = importlib.util.spec_from_file_location("hil_protocol", sim / "arduino_hil" / "hil_protocol.py")
hp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hp)

cases = [
    (hp.SENSOR_IMU, 1, 1_700_000_000_000_000_000, hp.pack_imu(0.1, -0.2, 9.81, 0.01, -0.02, 0.03)),
    (hp.SENSOR_BARO, 2, 1_700_000_000_020_000_000, hp.pack_baro(1.234)),
    (hp.SENSOR_MAG, 3, 1_700_000_000_040_000_000, hp.pack_mag(6e-6, 2.3e-5, -4.2e-5)),
    (hp.SENSOR_GPS, 4, 1_700_000_000_060_000_000, hp.pack_gps(15.8497, 74.4977, 12.5)),
    (hp.ACTUATOR_COMMAND, 5, 123456789, hp.pack_motors(0.66, 0.65, 0.67, 0.0)),
    (hp.CONTROL_INPUT, 6, 987654321, hp.pack_control_input(0.0, 0.2, -0.3, 0.1)),
    (hp.HELLO, 7, 1, b"UAV_HIL_V1"),
    (hp.HELLO_ACK, 8, 2, b"UAV_HIL_V1"),
    (hp.START, 9, 3, b""),
    (hp.STOP, 10, 4, b""),
    (hp.HEARTBEAT, 4294967295, 18446744073709551615, b""),
]
out = {"source": str(sim / "arduino_hil" / "hil_protocol.py"), "packets": []}
for t, seq, ts, payload in cases:
    pk = hp.build_packet(t, seq, ts, payload)
    assert hp.parse_packet(pk)["type"] == t
    out["packets"].append({"type": t, "seq": seq, "ts": str(ts), "payload_hex": payload.hex(), "packet_hex": pk.hex()})
dest = root / "tests" / "golden_vectors.json"
dest.write_text(json.dumps(out, indent=1), encoding="utf-8")
print("wrote", dest, len(out["packets"]), "packets")
