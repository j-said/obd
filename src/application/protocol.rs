use serde::{Deserialize, Serialize};

#[derive(Deserialize, Debug)]
#[serde(rename_all = "snake_case")]
pub enum Command {
    GetVin,
    ClearDtcs,
    GetLiveData { pid: u8 },
    GetStoredDtcs,
}

#[derive(Serialize)]
pub struct Response<T: Serialize> {
    pub id: u32,
    pub status: Status,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<T>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub debug: Option<DebugMsg>,
}

#[derive(Serialize, Clone, Copy)]
#[serde(rename_all = "snake_case")]
pub enum DebugMsg {
    ObdTimeout,
    LiveDataFailed,
    InvalidFormat,
    GetStoredDtcsFailed,
}

#[derive(Deserialize, Debug)]
pub struct Request {
    pub id: u32,
    pub cmd: Command,
}

#[derive(Serialize, Clone, Copy)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Status {
    Ok,
    Error,
}

// TODO: Consider adding more detailed error codes or messages in the future for better debugging and user feedback.
// TODO: Implement support for additional OBD-II commands and responses as needed in the future, such as Service 0x09 for more PIDs, Service 0x0A for permanent DTCs, etc.
// TODO: Add support for sending physical requests to specific ECUs, not just functional requests, if needed in the future.

// TODO: Think about switching to a more compact binary protocol