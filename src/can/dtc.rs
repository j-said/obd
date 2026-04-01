use super::iso_tp::EcuResponse;
use core::fmt::Write;
use heapless::{String, Vec};

#[derive(Debug, Clone)]
pub struct ParsedEcuDtcs {
    pub ecu_id: u32,
    pub dtcs: Vec<String<5>, 32>,
}

pub fn decode_responses(responses: &[EcuResponse]) -> Vec<ParsedEcuDtcs, 8> {
    let mut parsed = Vec::new();
    for resp in responses {
        parsed.push(decode_payload(resp)).ok();
    }
    parsed
}

fn decode_payload(resp: &EcuResponse) -> ParsedEcuDtcs {
    let mut dtcs = Vec::new();

    let start_idx = if !resp.data.is_empty() && resp.data[0] == 0x43 {
        1
    } else {
        0
    };
    let payload = &resp.data[start_idx..];

    for chunk in payload.chunks_exact(2) {
        if chunk[0] == 0 && chunk[1] == 0 {
            continue;
        }
        dtcs.push(parse_bytes(chunk[0], chunk[1])).ok();
    }

    ParsedEcuDtcs {
        ecu_id: resp.id,
        dtcs,
    }
}

fn parse_bytes(b1: u8, b2: u8) -> String<5> {
    let mut dtc = String::<5>::new();

    let prefix = match b1 >> 6 {
        0 => 'P',
        1 => 'C',
        2 => 'B',
        _ => 'U',
    };

    let d1 = (b1 >> 4) & 0x03;
    let d2 = b1 & 0x0F;
    let d3 = b2 >> 4;
    let d4 = b2 & 0x0F;

    write!(dtc, "{}{:X}{:X}{:X}{:X}", prefix, d1, d2, d3, d4).ok();
    dtc
}


// TODO: Consider adding support for OBD-II over CAN FD if needed in the future.
// TODO: Add unit tests for the decode_responses and parse_bytes functions to ensure correct DTC parsing.
// TODO: Implement logging for debugging purposes, especially for cases where the ECU response is unexpected or malformed.
// TODO: Add support for parsing permanent DTCs (Service 0x0A) and pending DTCs (Service 0x02) if needed in the future.