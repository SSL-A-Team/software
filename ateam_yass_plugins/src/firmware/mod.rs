use std::{
    ffi::{c_char, c_void},
    ptr::null_mut,
};
use yass_corelib::*;

#[unsafe(no_mangle)]
pub extern "C" fn yass_load_ateam_firmware(
    color: TeamColor,
    config: *const c_char,
) -> *mut c_void {
    println!("A-Team firmware loaded!");
    null_mut()
}

#[unsafe(no_mangle)]
pub extern "C" fn yass_run_ateam_firmware(
    state: SimulationState,
    delta_t_us: u64,
    user_data: *mut c_void,
    commands_out: &mut [RobotCommand; 16],
) {

}

#[unsafe(no_mangle)]
pub extern "C" fn yass_unload_ateam_firmware(user_data: *mut c_void) {

}