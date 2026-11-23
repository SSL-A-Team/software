use crate::firmware::{
    firmware_config::FirmwareConfig, motion_controller::MotionController,
    radio_connection::RadioConnection,
};
use ateam_common_packets::bindings::KickRequest;
use yass_corelib::{DribblerCommand, Robot, RobotCommand, SimulationState, TeamColor};

pub struct FirmwarePlugin {
    color: TeamColor,
    connections: [Option<RadioConnection>; 16],
    motion_controllers: [MotionController; 16],
}

impl FirmwarePlugin {
    pub fn new(
        color: TeamColor,
        _config: FirmwareConfig,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            color,
            connections: [const { None }; 16],
            motion_controllers: std::array::from_fn(|_| MotionController::new()),
        })
    }

    pub fn run(
        &mut self,
        state: &SimulationState,
        delta_t_us: u64,
        commands_out: &mut [RobotCommand; 16],
    ) {
        let robots = match self.color {
            TeamColor::Yellow => state.yellow_robots,
            TeamColor::Blue => state.blue_robots,
        };
        for i in 0..robots.len() {
            self.run_single_bot(robots[i], delta_t_us, &mut commands_out[i]);
        }
    }

    fn run_single_bot(&mut self, robot: Robot, delta_t_us: u64, command_out: &mut RobotCommand) {
        if !robot.present {
            self.ensure_disconnected(robot.id);
            return;
        }
        if !self.ensure_connected(&robot) {
            self.motion_controllers[robot.id as usize].clear_active_command();
            return;
        }
        let Some(conn) = &mut self.connections[robot.id as usize] else {
            return;
        };
        let should_send_feedback = match conn.update() {
            Ok(b) => b,
            Err(e) => {
                eprintln!("Error updaing radio connection: {e}");
                return;
            }
        };
        if let Some(command) = conn.get_command() {
            self.motion_controllers[robot.id as usize].set_active_command(command);
            self.motion_controllers[robot.id as usize].update(delta_t_us, command_out);
            command_out.active = true;
            command_out.kick_command.speed = match command.kick_request {
                KickRequest::KR_KICK_NOW
                | KickRequest::KR_KICK_CAPTURED
                | KickRequest::KR_KICK_TOUCH => command.kick_vel,
                _ => 0.0,
            };
            command_out.kick_command.angle = 0.0;
            command_out.dribbler_command = match command.dribbler_mode {
                ateam_common_packets::bindings::DribblerCommand::DC_CURRENT => {
                    if command.dribbler_setpoint > 0.0 {
                        DribblerCommand::Sticky
                    } else {
                        DribblerCommand::Off
                    }
                }
                _ => DribblerCommand::Off,
            }
        } else {
            self.motion_controllers[robot.id as usize].clear_active_command();
            command_out.active = false;
        }
        if should_send_feedback {
            let mode = self.motion_controllers[robot.id as usize].get_radio_body_control_mode();
            let control_telemetry = self.motion_controllers[robot.id as usize].get_radio_control_feedback();
            if let Err(e) = conn.send_feedback(&robot, mode, control_telemetry) {
                eprintln!("Error sending feedback: {e}");
            }
        }
    }

    fn ensure_connected(&mut self, robot: &Robot) -> bool {
        if let Some(conn) = &mut self.connections[robot.id as usize] {
            if !conn.is_connected() {
                if let Err(e) = conn.update() {
                    eprintln!("Error updating radio connection during discovery: {e}");
                }
                return false;
            } else {
                return true;
            }
        } else {
            let maybe_conn = RadioConnection::new(self.color, robot.id);
            match maybe_conn {
                Ok(conn) => {
                    println!("Creating new connection");
                    self.connections[robot.id as usize] = Some(conn);
                }
                Err(e) => {
                    eprintln!("Error creating robot connection: {e}");
                }
            }
            return false;
        }
    }

    fn ensure_disconnected(&mut self, id: u8) {
        if self.connections[id as usize].is_some() {
            self.connections[id as usize] = None;
        }
    }
}
