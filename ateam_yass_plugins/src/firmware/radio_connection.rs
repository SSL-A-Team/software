use ateam_common_packets::bindings::{
    BasicControl, BasicTelemetry, BodyControlTelemetry, CommandCode, HelloRequest, HelloResponse,
    RadioData, RadioHeader, RadioPacket
};
use std::{
    io::ErrorKind,
    mem::offset_of,
    net::{SocketAddr, ToSocketAddrs, UdpSocket},
    time::{Duration, Instant},
};
use yass_corelib::{Robot, TeamColor};

const COMMS_GIT_SHA: &str = env!("COMMS_GIT_SHA");
const COMMS_GIT_DIRTY: &str = env!("COMMS_GIT_DIRTY");

pub struct RadioConnection {
    socket: UdpSocket,
    color: TeamColor,
    id: u8,
    connected: bool,
    last_discovery_send_time: Instant,
    bridge_addr: Option<SocketAddr>,
    recv_buf: [u8; 2048],
    last_command: Option<BasicControl>,
}

impl RadioConnection {
    pub fn new(color: TeamColor, id: u8) -> Result<Self, Box<dyn std::error::Error>> {
        let socket = UdpSocket::bind("127.0.0.1:0")?;
        socket.set_nonblocking(true)?;
        Ok(RadioConnection {
            socket: socket,
            color,
            id,
            connected: false,
            last_discovery_send_time: Instant::now(),
            bridge_addr: None,
            recv_buf: [0; 2048],
            last_command: None,
        })
    }

    pub fn is_connected(&self) -> bool {
        self.connected
    }

    pub fn update(&mut self, sim_bot: &Robot) -> Result<(), Box<dyn std::error::Error>> {
        if !self.connected {
            self.update_discovering()
        } else {
            self.update_connected(sim_bot)
        }
    }

    fn update_discovering(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        loop {
            let Some(packet) = self.recv_packet()? else {
                break;
            };
            if packet.header.command_code != CommandCode::CC_HELLO_RESP
                || packet.header.data_length as usize != size_of::<HelloResponse>()
            {
                continue;
            }
            let response: HelloResponse = unsafe { packet.data.hello_response };
            self.bridge_addr = Some(SocketAddr::from((response.ipv4, response.port)));
            self.connected = true;
            return Ok(());
        }

        if self.last_discovery_send_time.elapsed() > Duration::from_secs(1) {
            self.send_discovery()?;
            self.last_discovery_send_time = Instant::now();
        }
        Ok(())
    }

    fn update_connected(&mut self, sim_bot: &Robot) -> Result<(), Box<dyn std::error::Error>> {
        loop {
            let Some(packet) = self.recv_packet()? else {
                break;
            };
            match packet.header.command_code {
                CommandCode::CC_GOODBYE => {
                    self.connected = false;
                    self.bridge_addr = None;
                    break;
                },
                CommandCode::CC_CONTROL => {
                    if packet.header.command_code != CommandCode::CC_CONTROL
                        || packet.header.data_length as usize != size_of::<BasicControl>()
                    {
                        continue;
                    }
                    self.last_command = Some(unsafe { packet.data.control });
                    self.send_feedback(sim_bot)?;
                },
                _ => {},
            }
        }
        Ok(())
    }

    pub fn get_command(&mut self) -> Option<BasicControl> {
        self.last_command.take()
    }

    fn send_discovery(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let comms_repo_dirty = (COMMS_GIT_DIRTY == "1") as u8;
        let comms_repo_sha = u32::from_str_radix(&COMMS_GIT_SHA[..8], 16)?.to_be_bytes();
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                command_code: CommandCode::CC_HELLO_REQ,
                _reserved: 0,
                data_length: size_of::<HelloRequest>() as u16,
            },
            data: RadioData {
                hello_request: HelloRequest {
                    robot_id: self.id,
                    color: match self.color {
                        TeamColor::Yellow => 0,
                        TeamColor::Blue => 1,
                    },
                    _bitfield_align_1: Default::default(),
                    _bitfield_1: HelloRequest::new_bitfield_1(comms_repo_dirty, 0, 0, 0),
                    reserved: [0],
                    coms_hash: comms_repo_sha,
                    controls_hash: [0; 4],
                    firmware_hash: [0; 4],
                },
            },
        };
        println!("Sending discovery packet");
        self.send_packet::<HelloRequest, _>(packet, "224.4.20.69:42069")
    }

    fn send_feedback(&mut self, sim_bot: &Robot) -> Result<(), Box<dyn std::error::Error>> {
        let Some(bridge_addr) = self.bridge_addr else {
            return Ok(());
        };
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                command_code: CommandCode::CC_TELEMETRY,
                _reserved: 0,
                data_length: size_of::<BasicTelemetry>() as u16,
            },
            data: RadioData {
                telemetry: BasicTelemetry {
                    transmission_sequence_number: 0,
                    control_data_sequence_number: 0,
                    body_control_mode: 0,
                    reserved1: 0,
                    _bitfield_align_1: Default::default(),
                    _bitfield_1: BasicTelemetry::new_bitfield_1(
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0,
                    ),
                    battery_percent: 100,
                    kicker_charge_percent: 100,
                    control_telem: BodyControlTelemetry::default(),
                    kf_body_pos_estimate: [0, 0, 0],
                    kf_body_vel_estimate: [0, 0, 0],
                },
            },
        };
        self.send_packet::<BasicTelemetry, _>(packet, bridge_addr)
    }

    fn send_goodbye(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                command_code: CommandCode::CC_GOODBYE,
                _reserved: 0,
                data_length: 0,
            },
            data: unsafe { std::mem::zeroed() },
        };
        self.send_packet::<(), _>(packet, "")
    }

    fn send_packet<DataType, AddrType: ToSocketAddrs>(
        &mut self,
        packet: RadioPacket,
        dest: AddrType,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let packet_bytes = unsafe {
            std::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<DataType>(),
            )
        };
        self.socket.send_to(packet_bytes, dest)?;
        Ok(())
    }

    fn recv_packet(&mut self) -> Result<Option<RadioPacket>, Box<dyn std::error::Error>> {
        let bytes_read = match self.socket.recv(&mut self.recv_buf) {
            Ok(bytes_read) => bytes_read,
            Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                return Ok(None);
            }
            Err(e) => Err(e)?,
        };
        Ok(parse_packet(&self.recv_buf[..bytes_read]))
    }
}

impl Drop for RadioConnection {
    fn drop(&mut self) {
        if self.connected {
            let _ = self.send_goodbye();
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

fn parse_packet(bytes: &[u8]) -> Option<RadioPacket> {
    const HEADER_LEN: usize = offset_of!(RadioPacket, data);
    if bytes.len() < HEADER_LEN || bytes.len() > size_of::<RadioPacket>() {
        return None;
    }
    let mut packet: RadioPacket = unsafe { std::mem::zeroed() };
    unsafe {
        std::ptr::copy_nonoverlapping(
            bytes.as_ptr(),
            (&mut packet as *mut RadioPacket).cast::<u8>(),
            bytes.len(),
        );
    }
    if packet.header.data_length as usize != (bytes.len() - HEADER_LEN) {
        return None;
    }
    Some(packet)
}
