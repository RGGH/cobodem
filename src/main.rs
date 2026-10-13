//! mycobot_mock — a fake myCobot 280 TCP server, in Rust.
//!
//! This mimics what `Server_280.py` does over the wire, WITHOUT needing a
//! real robot or serial port. It's meant to sit in place of the Pi during
//! development so you can exercise your Rust client's framing, parsing,
//! and reconnect/timeout logic.
//!
//! Wire behaviour (one `recv()` == one message, like the real server):
//!
//!   1. Handshake: the client sends three plain-text messages
//!      (serial port, baud rate, timeout). The server replies b"1" to each.
//!
//!   2. Commands: the client sends each frame as the TEXT of a list, e.g.
//!      "[254, 254, 2, 32, 250]". Raw FE FE ... bytes are also accepted.
//!      The server replies with a raw binary frame.
//!
//! Frame shape (both directions):
//!
//!       FE FE  LEN  CMD  [DATA...]  FA
//!       \__/   |    |    |          |
//!       header |    |    payload    footer
//!              |    command id (ProtocolCode.*)
//!              LEN = len(DATA) + 2   (covers CMD byte + FA footer byte)
//!
//!   Angles are encoded as big-endian i16, value = round(degrees * 100).
//!   Coords (x,y,z,rx,ry,rz) are encoded as big-endian i16, value = round(v * 10).
//!   Simple booleans/status/speed values are a single raw byte.
//!
//! This is *not* a physics-accurate simulator — coords returned by
//! GET_COORDS are not a real forward-kinematics solution, they're just
//! plausible-looking numbers derived from the current angles so a client
//! doing a get/compare loop sees self-consistent, moving data.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;

const HEADER: u8 = 0xFE;
const FOOTER: u8 = 0xFA;

// A representative subset of ProtocolCode from pymycobot/common.py.
// Extend this as you add commands your Rust client actually sends.
mod cmd {
    pub const POWER_ON: u8 = 0x10;
    pub const POWER_OFF: u8 = 0x11;
    pub const IS_POWER_ON: u8 = 0x12;
    pub const RELEASE_ALL_SERVOS: u8 = 0x13;

    pub const GET_ANGLES: u8 = 0x20;
    pub const SEND_ANGLE: u8 = 0x21;
    pub const SEND_ANGLES: u8 = 0x22;
    pub const GET_COORDS: u8 = 0x23;
    pub const SEND_COORD: u8 = 0x24;
    pub const SEND_COORDS: u8 = 0x25;
    pub const PAUSE: u8 = 0x26;
    pub const IS_PAUSED: u8 = 0x27;
    pub const RESUME: u8 = 0x28;
    pub const STOP: u8 = 0x29;
    pub const IS_IN_POSITION: u8 = 0x2A;
    pub const IS_MOVING: u8 = 0x2B;

    pub const GET_SPEED: u8 = 0x40;
    pub const SET_SPEED: u8 = 0x41;
}

/// Shared robot state, guarded by a mutex since each connection gets its
/// own thread (mirrors Server_280.py handling one client at a time, but
/// this mock happens to allow several).
#[derive(Debug)]
struct RobotState {
    angles_deg: [f64; 6], // joint 1..6, degrees
    speed: u8,            // 0-100
    powered: bool,
    paused: bool,
}

impl RobotState {
    fn new() -> Self {
        RobotState {
            angles_deg: [0.0; 6],
            speed: 50,
            powered: true,
            paused: false,
        }
    }

    /// Fake coords derived from joint angles. NOT real forward kinematics —
    /// just enough to look like plausible, changing numbers.
    fn fake_coords(&self) -> [f64; 6] {
        let a = &self.angles_deg;
        [
            150.0 + a[0] * 0.8, // x (mm)
            a[1] * 0.8,         // y (mm)
            200.0 + a[2] * 0.5, // z (mm)
            a[3],               // rx (deg)
            a[4],               // ry (deg)
            a[5],               // rz (deg)
        ]
    }
}

fn angle_to_i16(deg: f64) -> i16 {
    (deg * 100.0).round() as i16
}

fn i16_to_angle(v: i16) -> f64 {
    (v as f64 / 100.0 * 1000.0).round() / 1000.0
}

fn coord_to_i16(v: f64) -> i16 {
    (v * 10.0).round() as i16
}

/// Build a full reply frame: FE FE LEN CMD DATA... FA
fn build_frame(command: u8, data: &[u8]) -> Vec<u8> {
    let len = (data.len() + 2) as u8;
    let mut out = Vec::with_capacity(data.len() + 5);
    out.push(HEADER);
    out.push(HEADER);
    out.push(len);
    out.push(command);
    out.extend_from_slice(data);
    out.push(FOOTER);
    out
}

fn ack() -> Vec<u8> {
    vec![0x01]
}

/// Parse one message into (command, data). Accepts either the text form
/// "[254, 254, 2, 32, 250]" (what the client sends) or raw FE FE ... bytes.
fn parse_message(buf: &[u8]) -> Option<(u8, Vec<u8>)> {
    let bytes: Vec<u8> = if buf.starts_with(&[HEADER, HEADER]) {
        buf.to_vec()
    } else {
        std::str::from_utf8(buf)
            .ok()?
            .trim()
            .trim_start_matches('[')
            .trim_end_matches(']')
            .split(',')
            .filter_map(|t| t.trim().parse::<u8>().ok())
            .collect()
    };
    if bytes.len() < 5 || bytes[0] != HEADER || bytes[1] != HEADER {
        return None;
    }
    let len = bytes[2] as usize; // CMD + DATA + FOOTER
    if len < 2 || bytes.len() < 3 + len {
        return None;
    }
    if bytes[3 + len - 1] != FOOTER {
        eprintln!(
            "warning: frame for cmd {:#04x} missing FA footer (got {:#04x}), using it anyway",
            bytes[3],
            bytes[3 + len - 1]
        );
    }
    let command = bytes[3];
    let data = bytes[4..3 + len - 1].to_vec();
    Some((command, data))
}

fn handle_client(mut stream: TcpStream, state: Arc<Mutex<RobotState>>) {
    let peer = stream
        .peer_addr()
        .map(|a| a.to_string())
        .unwrap_or_else(|_| "unknown".into());
    println!("client connected: {peer}");

    let mut buf = [0u8; 1024];

    // Handshake: serial port, baud, timeout -> reply "1" to each.
    for step in ["serial port", "baud", "timeout"] {
        match stream.read(&mut buf) {
            Ok(0) | Err(_) => {
                println!("client {peer} dropped during handshake");
                return;
            }
            Ok(n) => {
                println!("handshake {step}: {:?}", String::from_utf8_lossy(&buf[..n]));
                if stream.write_all(b"1").is_err() {
                    return;
                }
            }
        }
    }

    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) => {
                println!("client disconnected: {peer}");
                return;
            }
            Ok(n) => n,
            Err(e) => {
                eprintln!("read error from {peer}: {e}");
                return;
            }
        };
        let Some((command, data)) = parse_message(&buf[..n]) else {
            eprintln!("unparseable message from {peer}: {:?}", &buf[..n]);
            continue;
        };

        let reply_data: Vec<u8> = {
            let mut st = state.lock().unwrap();
            match command {
                cmd::POWER_ON => {
                    st.powered = true;
                    ack()
                }
                cmd::POWER_OFF => {
                    st.powered = false;
                    ack()
                }
                cmd::IS_POWER_ON => vec![st.powered as u8],
                cmd::RELEASE_ALL_SERVOS => ack(),

                cmd::GET_ANGLES => {
                    let mut out = Vec::with_capacity(12);
                    for a in st.angles_deg.iter() {
                        out.extend_from_slice(&angle_to_i16(*a).to_be_bytes());
                    }
                    out
                }
                cmd::SEND_ANGLE => {
                    // payload: joint_id(1) angle_i16(2) speed(1)
                    if data.len() >= 4 {
                        let joint = data[0] as usize;
                        let raw = i16::from_be_bytes([data[1], data[2]]);
                        let speed = data[3];
                        if (1..=6).contains(&joint) {
                            st.angles_deg[joint - 1] = i16_to_angle(raw);
                        }
                        st.speed = speed;
                    }
                    ack()
                }
                cmd::SEND_ANGLES => {
                    // payload: 6x angle_i16(2) speed(1) = 13 bytes
                    if data.len() >= 13 {
                        for j in 0..6 {
                            let raw = i16::from_be_bytes([data[j * 2], data[j * 2 + 1]]);
                            st.angles_deg[j] = i16_to_angle(raw);
                        }
                        st.speed = data[12];
                    }
                    ack()
                }
                cmd::GET_COORDS => {
                    let mut out = Vec::with_capacity(12);
                    for c in st.fake_coords().iter() {
                        out.extend_from_slice(&coord_to_i16(*c).to_be_bytes());
                    }
                    out
                }
                cmd::SEND_COORD | cmd::SEND_COORDS => {
                    // We don't track coords as independent state (they're
                    // derived from angles), so just accept + ack. Extend
                    // this if your client needs coord moves reflected back.
                    ack()
                }
                cmd::PAUSE => {
                    st.paused = true;
                    ack()
                }
                cmd::IS_PAUSED => vec![st.paused as u8],
                cmd::RESUME => {
                    st.paused = false;
                    ack()
                }
                cmd::STOP => ack(),
                cmd::IS_IN_POSITION => vec![1u8],
                cmd::IS_MOVING => vec![0u8],

                cmd::GET_SPEED => vec![st.speed],
                cmd::SET_SPEED => {
                    if let Some(&s) = data.first() {
                        st.speed = s;
                    }
                    ack()
                }

                other => {
                    eprintln!(
                        "unhandled cmd {other:#04x} (len={}) from {peer} — replying with generic ack",
                        data.len()
                    );
                    ack()
                }
            }
        };

        let frame = build_frame(command, &reply_data);
        if let Err(e) = stream.write_all(&frame) {
            eprintln!("write error to {peer}: {e}");
            return;
        }
    }
}

fn main() -> std::io::Result<()> {
    let port = std::env::args()
        .nth(1)
        .and_then(|s| s.parse::<u16>().ok())
        .unwrap_or(9000);

    let listener = TcpListener::bind(("0.0.0.0", port))?;
    println!("mycobot_mock listening on 0.0.0.0:{port}");
    println!("(point your Rust client at 127.0.0.1:{port} instead of the Pi's IP)");

    let state = Arc::new(Mutex::new(RobotState::new()));

    for stream in listener.incoming() {
        match stream {
            Ok(s) => {
                let state = Arc::clone(&state);
                thread::spawn(move || handle_client(s, state));
            }
            Err(e) => eprintln!("accept error: {e}"),
        }
    }

    Ok(())
}
