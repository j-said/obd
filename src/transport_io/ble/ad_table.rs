use super::error::BleError;
use super::types::DEVICE_NAME;
use trouble_host::advertise::{AdStructure, Advertisement};
use trouble_host::prelude::{BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE};

/// Кодує дані реклами у наданий буфер і повертає об'єкт Advertisement.
///
/// `buf` — буфер, куди будуть записані сирі байти (default має бути >= 31 байт).
pub fn create_advertisement<'a>(buf: &'a mut [u8]) -> Result<Advertisement<'a>, BleError> {
    let structures = [
        AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
        AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes()),
    ];

    let len =
        AdStructure::encode_slice(&structures, buf).map_err(|_| BleError::AdvertisingError)?;

    Ok(Advertisement::ConnectableScannableUndirected {
        adv_data: &buf[..len],
        scan_data: &[],
    })
}
