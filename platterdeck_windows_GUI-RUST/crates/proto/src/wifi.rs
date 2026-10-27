//! Wi-Fi v2: one 40-byte UDP datagram, port 19704, 500 Hz telemetry.
//! 0..4 PDW2; 4 kind (1 command, 2 telemetry); 5..8 zero; 8..12 host session;
//! 12..16 sequence; 16..20 sample timestamp (us, echoed by commands);
//! 20..24 board boot ID; 24..36 v1 payload; 36..40 IEEE CRC32 of 0..36.
//! All integers little endian. FREE with boot=0 is the discovery handshake.
use crate::{crc8, FRAME_LEN, SYNC};

pub const DEFAULT_ADDR: &str = "192.168.4.1:19704";
pub const PACKET_LEN: usize = 40;
pub const COMMAND: u8 = 1;
pub const TELEMETRY: u8 = 2;
pub const HEARTBEAT_MS: u64 = 20;
/// Link timeout on both sides: the board coasts and the PC declares the
/// link down after this much silence. Long enough to ride out a Windows
/// Wi-Fi background scan, short enough that a dead PC never leaves the
/// platter driven for long.
pub const TIMEOUT_US: u64 = 500_000;

pub fn newer(a: u32, b: u32) -> bool {
    a != b && a.wrapping_sub(b) < 0x8000_0000
}
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ if crc & 1 != 0 { 0xedb8_8320 } else { 0 };
        }
    }
    !crc
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Packet {
    pub kind: u8,
    pub session: u32,
    pub seq: u32,
    pub time_us: u32,
    pub boot: u32,
    pub payload: [u8; FRAME_LEN],
}
impl Packet {
    pub fn encode(&self) -> [u8; PACKET_LEN] {
        let mut b = [0u8; PACKET_LEN];
        b[..4].copy_from_slice(b"PDW2");
        b[4] = self.kind;
        b[8..12].copy_from_slice(&self.session.to_le_bytes());
        b[12..16].copy_from_slice(&self.seq.to_le_bytes());
        b[16..20].copy_from_slice(&self.time_us.to_le_bytes());
        b[20..24].copy_from_slice(&self.boot.to_le_bytes());
        b[24..36].copy_from_slice(&self.payload);
        let crc = crc32(&b[..36]);
        b[36..].copy_from_slice(&crc.to_le_bytes());
        b
    }
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() != PACKET_LEN
            || &b[..4] != b"PDW2"
            || !matches!(b[4], COMMAND | TELEMETRY)
            || b[5..8] != [0; 3]
            || crc32(&b[..36]) != u32::from_le_bytes(b[36..40].try_into().ok()?)
            || b[24] != SYNC
            || crc8(&b[24..35]) != b[35]
        {
            return None;
        }
        let p = Self {
            kind: b[4],
            session: u32::from_le_bytes(b[8..12].try_into().ok()?),
            seq: u32::from_le_bytes(b[12..16].try_into().ok()?),
            time_us: u32::from_le_bytes(b[16..20].try_into().ok()?),
            boot: u32::from_le_bytes(b[20..24].try_into().ok()?),
            payload: b[24..36].try_into().ok()?,
        };
        if p.session == 0 || (p.kind == COMMAND && (p.payload[2] > 3 || p.payload[9..11] != [0, 0]))
        {
            return None;
        }
        Some(p)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn envelope_checks_all_bytes_and_length() {
        assert_eq!(crc32(b"123456789"), 0xcbf43926);
        let p = Packet {
            kind: COMMAND,
            session: 0x12345678,
            seq: 42,
            time_us: 1234,
            boot: 99,
            payload: crate::HostFrame::default().encode(),
        };
        let b = p.encode();
        assert_eq!(Packet::decode(&b), Some(p));
        // Same independently calculated fixture used by the C++ firmware test.
        assert_eq!(
            b,
            [
                0x50, 0x44, 0x57, 0x32, 0x01, 0, 0, 0, 0x78, 0x56, 0x34, 0x12, 0x2a, 0, 0, 0, 0xd2,
                0x04, 0, 0, 0x63, 0, 0, 0, 0xaa, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x82, 0x03, 0xba,
                0x4b, 0x96
            ]
        );
        for i in 0..PACKET_LEN {
            let mut corrupt = b;
            corrupt[i] ^= 1;
            assert!(Packet::decode(&corrupt).is_none());
        }
        for n in 0..PACKET_LEN {
            assert!(Packet::decode(&b[..n]).is_none());
        }
        let mut long = [0; PACKET_LEN + 1];
        long[..PACKET_LEN].copy_from_slice(&b);
        assert!(Packet::decode(&long).is_none());
    }
    #[test]
    fn ordering_wrap_duplicates_and_old_packets() {
        assert!(newer(0, u32::MAX));
        assert!(newer(12, 10));
        assert!(!newer(10, 10));
        assert!(!newer(9, 10));
        assert!(!newer(u32::MAX, 0));
    }
}
