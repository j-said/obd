use serde::{Deserialize, Serialize};

#[derive(Deserialize, Debug)]
#[serde(rename_all = "snake_case")]
pub enum Command {
    GetVin,
    ClearDtcs,
    GetLiveData { pid: u8 },
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

#[derive(Serialize, Clone, Copy)]
#[serde(rename_all = "snake_case")]
pub enum DebugMsg {
    ObdTimeout,
    LiveDataFailed,
    InvalidFormat,
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