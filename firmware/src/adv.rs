//! Lamp state broadcast in BLE advertisements, so one sender can control many lamps at once.
//!
//! The state travels in a manufacturer-specific AD structure:
//!
//! ```text
//! [0x0B, 0xFF, 0xFF, 0xFF, 'C', 'L', version, group, seq, mode, brightness, mix]
//!   len  type  company id   magic
//! ```
//!
//! - company id `0xFFFF` is reserved by the Bluetooth SIG for testing
//! - `group`: [`GROUP_ALL`] addresses every lamp (lamp groups are not implemented yet)
//! - `seq`: lamps apply a broadcast only when the sequence number changes, so a sender can
//!   repeat the same packet forever without overriding local knob changes
//! - `mode`: settings encoding (white=0, uv=1, ir=2, spread=3); `brightness`, `mix`: 0..=40
//!
//! Every packet carries the full state, so a lamp that missed packets catches up with the next one.

use crate::protocol::MAX_LEVEL;

pub const COMPANY_ID: u16 = 0xFFFF;
pub const MAGIC: [u8; 2] = *b"CL";
pub const VERSION: u8 = 1;
pub const GROUP_ALL: u8 = 0;
const AD_TYPE_MANUFACTURER: u8 = 0xFF;
const PAYLOAD_LEN: usize = 8;
/// Length of the encoded AD structure, including its length byte.
pub const AD_LEN: usize = 4 + PAYLOAD_LEN;
const MAX_MODE: u8 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Broadcast {
    pub group: u8,
    pub seq: u8,
    pub mode: u8,
    pub brightness: u8,
    pub mix: u8,
}

impl Broadcast {
    /// The manufacturer-specific AD structure, ready to put into advertising data.
    pub fn encode(&self) -> [u8; AD_LEN] {
        let [company_lo, company_hi] = COMPANY_ID.to_le_bytes();
        [
            (AD_LEN - 1) as u8,
            AD_TYPE_MANUFACTURER,
            company_lo,
            company_hi,
            MAGIC[0],
            MAGIC[1],
            VERSION,
            self.group,
            self.seq,
            self.mode,
            self.brightness,
            self.mix,
        ]
    }

    /// Find a valid lamp broadcast in advertising (or scan response) data.
    pub fn parse(adv_data: &[u8]) -> Option<Self> {
        let mut rest = adv_data;
        while let [len, tail @ ..] = rest {
            let len = *len as usize;
            if len == 0 || len > tail.len() {
                return None; // padding or malformed data
            }
            let (structure, next) = tail.split_at(len);
            if let Some(broadcast) = Self::parse_structure(structure) {
                return Some(broadcast);
            }
            rest = next;
        }
        None
    }

    fn parse_structure(structure: &[u8]) -> Option<Self> {
        let [AD_TYPE_MANUFACTURER, company_lo, company_hi, payload @ ..] = structure else {
            return None;
        };
        if u16::from_le_bytes([*company_lo, *company_hi]) != COMPANY_ID || payload.len() < PAYLOAD_LEN {
            return None;
        }
        let [m0, m1, version, group, seq, mode, brightness, mix, ..] = *payload else {
            return None;
        };
        if [m0, m1] != MAGIC || version != VERSION || mode > MAX_MODE || brightness > MAX_LEVEL || mix > MAX_LEVEL {
            return None;
        }
        Some(Self { group, seq, mode, brightness, mix })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: Broadcast = Broadcast { group: GROUP_ALL, seq: 7, mode: 3, brightness: 25, mix: 10 };

    #[test]
    fn round_trip() {
        assert_eq!(Broadcast::parse(&SAMPLE.encode()), Some(SAMPLE));
    }

    #[test]
    fn found_between_other_structures() {
        let mut data = [0u8; 31];
        data[..3].copy_from_slice(&[0x02, 0x01, 0x06]); // flags
        data[3..3 + AD_LEN].copy_from_slice(&SAMPLE.encode());
        // the rest is zero padding, which ends the data
        assert_eq!(Broadcast::parse(&data), Some(SAMPLE));
    }

    #[test]
    fn other_manufacturer_data_and_garbage_are_ignored() {
        let mut other = SAMPLE.encode();
        other[2] = 0xD7; // another company id
        assert_eq!(Broadcast::parse(&other), None);
        let mut bad_magic = SAMPLE.encode();
        bad_magic[4] = b'X';
        assert_eq!(Broadcast::parse(&bad_magic), None);
        let mut too_bright = SAMPLE.encode();
        too_bright[10] = MAX_LEVEL + 1;
        assert_eq!(Broadcast::parse(&too_bright), None);
        let mut bad_mode = SAMPLE.encode();
        bad_mode[9] = 4;
        assert_eq!(Broadcast::parse(&bad_mode), None);
        assert_eq!(Broadcast::parse(&[0x05, 0xFF, 0xFF]), None); // truncated
        assert_eq!(Broadcast::parse(&[]), None);
    }
}
