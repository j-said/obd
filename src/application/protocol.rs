use serde::{Deserialize, Serialize};

#[derive(Deserialize, Debug)]
#[serde(rename_all = "snake_case")]
pub enum Command {
    GetVin,
    ClearDtcs,
    GetLiveData { pid: u8 },
    GetStoredDtcs,
    GetPendingDtcs,
    GenericRequest,
}

#[derive(Deserialize, Debug)]
pub struct Request {
    pub id: u32,
    pub cmd: Command,
}

#[derive(Serialize)]
pub struct Response<'a, T: Serialize> {
    pub id: u32,
    pub status: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<T>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub debug: Option<&'a str>,
}
