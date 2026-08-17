# import socket, struct

# s = socket.create_connection(("127.0.0.1", 9000), timeout=3)

# def send_frame(cmd, data=b""):
    # length = len(data) + 2
    # s.sendall(bytes([0xFE, 0xFE, length, cmd]) + data + bytes([0xFA]))

# def read_frame():
    # assert s.recv(2) == b"\xfe\xfe"
    # length = s.recv(1)[0]
    # rest = b""
    # while len(rest) < length:
        # rest += s.recv(length - len(rest))
    # return rest[0], rest[1:-1]  # cmd, data

# # GET_ANGLES (0x20) — no payload
# send_frame(0x20)
# cmd, data = read_frame()
# print([a / 100 for a in struct.unpack(">6h", data)])
import socket, struct, time

HOST, PORT = "127.0.0.1", 9000
s = socket.create_connection((HOST, PORT), timeout=3)


def send_frame(cmd, data=b""):
    length = len(data) + 2
    s.sendall(bytes([0xFE, 0xFE, length, cmd]) + data + bytes([0xFA]))


def read_frame():
    assert s.recv(2) == b"\xfe\xfe"
    length = s.recv(1)[0]
    rest = b""
    while len(rest) < length:
        rest += s.recv(length - len(rest))
    return rest[0], rest[1:-1]  # cmd, data


def roundtrip(name, cmd, data=b"", fmt=None, expect=None, tol=0.01):
    """Send a frame, print the reply, and optionally assert on the parsed value.

    expect can be:
      - a single value (compared to parsed[0] if fmt gives a tuple of len 1,
        or to parsed itself otherwise)
      - a sequence (compared element-wise, with tol for floats)
    """
    send_frame(cmd, data)
    rcmd, rdata = read_frame()
    if fmt:
        parsed = struct.unpack(fmt, rdata)
    else:
        parsed = list(rdata)
    print(f"{name:<20} cmd={rcmd:#04x} raw={rdata.hex()} -> {parsed}")

    if expect is not None:
        got = parsed
        want = expect if hasattr(expect, "__iter__") else (expect,)
        assert len(got) == len(want), (
            f"{name}: expected {len(want)} values, got {len(got)} ({got})"
        )
        for g, w in zip(got, want):
            if isinstance(w, float):
                assert abs(g - w) <= tol, f"{name}: expected ~{w}, got {g}"
            else:
                assert g == w, f"{name}: expected {w}, got {g}"
    return rdata


# --- power / status ---
roundtrip("POWER_ON", 0x10, expect=[0x01])
roundtrip("IS_POWER_ON", 0x12, expect=[True])

# --- baseline angles/coords (fresh server state = all zeros) ---
roundtrip("GET_ANGLES", 0x20, fmt=">6h", expect=[0, 0, 0, 0, 0, 0])
roundtrip("GET_COORDS", 0x23, fmt=">6h", expect=[1500, 0, 2000, 0, 0, 0])

# --- move a single joint: joint 3 to 45.00 deg at speed 60 ---
payload = bytes([3]) + struct.pack(">h", int(45.00 * 100)) + bytes([60])
roundtrip("SEND_ANGLE j3", 0x21, payload, expect=[0x01])

# isolate the effect of SEND_ANGLE before touching state again with SEND_ANGLES
roundtrip(
    "GET_ANGLES (post j3)",
    0x20,
    fmt=">6h",
    expect=[0, 0, 4500, 0, 0, 0],  # raw i16 counts = degrees * 100
)
roundtrip("GET_SPEED (post j3)", 0x40, expect=[60])

# --- move all joints at once, speed 40 ---
angles = [10.0, -20.0, 45.0, 0.0, 15.0, -5.0]
payload = b"".join(struct.pack(">h", int(a * 100)) for a in angles) + bytes([40])
roundtrip("SEND_ANGLES", 0x22, payload, expect=[0x01])

# confirm state actually changed server-side
roundtrip("GET_ANGLES (after)", 0x20, fmt=">6h", expect=[1000, -2000, 4500, 0, 1500, -500])
roundtrip(
    "GET_COORDS (after)",
    0x23,
    fmt=">6h",
    # fake_coords(): x=150+a0*0.8, y=a1*0.8, z=200+a2*0.5, rx=a3, ry=a4, rz=a5 (then *10, i16)
    expect=[
        round((150.0 + 10.0 * 0.8) * 10),
        round((-20.0 * 0.8) * 10),
        round((200.0 + 45.0 * 0.5) * 10),
        0,
        150,
        -50,
    ],
)

# --- speed get/set ---
roundtrip("GET_SPEED", 0x40, expect=[40])
roundtrip("SET_SPEED", 0x41, bytes([75]), expect=[0x01])
roundtrip("GET_SPEED (after)", 0x40, expect=[75])

# --- pause / resume ---
roundtrip("PAUSE", 0x26, expect=[0x01])
roundtrip("IS_PAUSED", 0x27, expect=[True])
roundtrip("RESUME", 0x28, expect=[0x01])
roundtrip("IS_PAUSED (after)", 0x27, expect=[False])

# --- motion status ---
roundtrip("IS_MOVING", 0x2B, expect=[0])
roundtrip("IS_IN_POSITION", 0x2A, expect=[1])

# --- unhandled command, to exercise the server's fallback-ack path ---
roundtrip("UNKNOWN 0x99", 0x99, b"\x01\x02\x03", expect=[0x01])

# --- rapid-fire stress: hammer GET_ANGLES to check for framing bugs ---
t0 = time.time()
for i in range(200):
    send_frame(0x20)
    rcmd, rdata = read_frame()
    assert rcmd == 0x20, f"stress loop iter {i}: got cmd {rcmd:#04x} instead of 0x20"
    assert len(rdata) == 12, f"stress loop iter {i}: bad payload length {len(rdata)}"
print(f"200x GET_ANGLES round trips OK in {time.time() - t0:.3f}s")

s.close()
print("\nAll checks passed.")

