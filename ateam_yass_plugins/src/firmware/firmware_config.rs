use serde::Deserialize;
use std::ffi::{CStr, c_char};

#[derive(Debug, Deserialize, Clone, Copy)]
#[serde(default)]
pub struct FirmwareConfig {}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {}
    }
}

pub fn parse_config(
    config_raw: *const c_char,
) -> Result<FirmwareConfig, Box<dyn std::error::Error>> {
    if config_raw.is_null() {
        return Ok(FirmwareConfig::default());
    }
    let text = unsafe { CStr::from_ptr(config_raw) }.to_str()?;
    Ok(serde_yaml::from_str(text)?)
}
