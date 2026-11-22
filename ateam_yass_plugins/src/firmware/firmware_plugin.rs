use crate::firmware::{firmware_config::FirmwareConfig, radio_connection::RadioConnection};
use yass_corelib::{Robot, RobotCommand, SimulationState, TeamColor};

pub struct FirmwarePlugin {
    color: TeamColor,
    connections: [Option<RadioConnection>; 16],
}

impl FirmwarePlugin {
    pub fn new(
        color: TeamColor,
        _config: FirmwareConfig,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            color,
            connections: [const { None }; 16],
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
            return;
        }
        let Some(conn) = &mut self.connections[robot.id as usize] else {
            return;
        };
        if let Err(e) = conn.update(&robot) {
            eprintln!("Error updaing radio connection: {e}");
            return;
        }
        if let Some(command) = conn.get_command() {
            // update active manuever
        }
        // TODO: execute active manuever
    }

    fn ensure_connected(&mut self, robot: &Robot) -> bool {
        if let Some(conn) = &mut self.connections[robot.id as usize] {
            if !conn.is_connected() {
                if let Err(e) = conn.update(robot) {
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
