use ateam_common_packets::bindings::{
    BasicControl, BodyControlMode, BodyControlTelemetry, LocalVelocityCommand, LocalVelocityTelemetry,
};
use yass_corelib::RobotCommand;

pub struct MotionController {
    active_command: Option<BasicControl>,
    telemetry: BodyControlTelemetry,
}

impl MotionController {
    pub fn new() -> Self {
        Self {
            active_command: None,
            telemetry: BodyControlTelemetry::default(),
        }
    }

    pub fn set_active_command(&mut self, command: BasicControl) {
        self.active_command = Some(command);
    }

    pub fn clear_active_command(&mut self) {
        self.active_command = None;
    }

    pub fn get_radio_body_control_mode(&self) -> BodyControlMode::Type {
        match self.active_command {
            Some(cmd) => cmd.body_control_mode,
            None => BodyControlMode::BCM_OFF,
        }
    }

    pub fn get_radio_control_feedback(&self) -> BodyControlTelemetry {
        BodyControlTelemetry::default()
    }

    pub fn update(&mut self, _delta_t_us: u64, command_out: &mut RobotCommand) {
        if let Some(active_command) = self.active_command {
            match active_command.body_control_mode {
                BodyControlMode::BCM_LOCAL_VELOCITY => {
                    command_out.motion_command_frame = yass_corelib::MotionCommandFrame::Local;
                    let cmd: LocalVelocityCommand = unsafe { active_command.cmd.local_vel };
                    command_out.motion_command = yass_corelib::MotionCommand::Velocity {
                        x: cmd.local_xd,
                        y: cmd.local_yd,
                        theta: cmd.local_omega,
                    };
                    self.telemetry.local_vel = LocalVelocityTelemetry::default();
                },
                _ => {
                    command_out.motion_command = yass_corelib::MotionCommand::Off;
                }
            }
        } else {
            command_out.motion_command = yass_corelib::MotionCommand::Off;
            self.telemetry = BodyControlTelemetry::default();
        }
    }
}
