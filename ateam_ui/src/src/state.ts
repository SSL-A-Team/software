import ROSLIB from "roslib"
// import 'roslib/build/roslib';
import { Team, TeamColor } from "@/team"
import { Overlay } from "@/overlay"
import { Referee } from "@/referee"
import { Ball } from "@/ball"
import { Field } from "@/field"
import { AIState } from "@/AI"
import { Play } from "@/play"
import { exportDefaultSpecifier } from "@babel/types"

export class RenderConfig {
    angle: number = 0; // Rotation applied to the rendered field
    scale: number = 140; // Pixels per meter (in the rendering canvas)   
}

export class WorldState {
    team_name: string;
    team: TeamColor;
    teams: Team[];
    ball: Ball;
    field: Field;
    referee: Referee;
    ai: AIState;
    ignore_side: number;

    constructor() {
        this.team_name = "A-Team";
        this.team = TeamColor.Blue;
        this.teams = [];
        this.teams[TeamColor.Blue] = new Team(this.team_name, TeamColor.Blue, -1);
        this.teams[TeamColor.Yellow] = new Team("Opponent", TeamColor.Yellow, 1);

        this.ball = new Ball();
        this.field = new Field();

        this.referee = new Referee();
        this.ai = new AIState();

        this.ignore_side = 0;
    }
}

export class AppState {
    renderConfig: RenderConfig;
    world: WorldState;

    // I'm not sure if we will need ongoing access to these
    ros: ROSLIB.Ros;
    publishers: ROSLIB.Topic[]
    subscriptions: ROSLIB.Topic[]
    services: ROSLIB.Service[]
    params: ROSLIB.Param[]

    sim: boolean = true;
    comp: boolean = false; // TODO: find a better way to set this value

    controlled_robot: number = null;

    plays = {};
    selected_play_name: string = null;
    override_play_in_progress: string = null;

    hovered_field_ignore_side: number = 0;
    draggedRobot: number = null;

    goalie_service_status: [status: boolean, reason: string] = [true, ""] // False if the set goalie service fails

    setGoalie(goalie_id: number) {
        const state = this; // fix dumb javascript things

        const request = new ROSLIB.ServiceRequest({
            desired_keeper: goalie_id
        });

        this.services["setGoalie"].callService(request,
            // Service Response Callback
            function(result) {
                if(!result.success) {
                    state.goalie_service_status = [false, result.reason];
                } else {
                    state.goalie_service_status = [true, ""];
                }
            },
            // Failed to call service callback
            function(result) {
                state.goalie_service_status = [false, result];
            }
        );
    }

    getGoalie(): string {
        if (this.world.referee && this.world.referee[this.world.team]) {
            const id = this.world.referee[this.world.team].goalkeeper
            if (id == null || isNaN(id)) {
                return "X";
            }

            return String(id);
        }

        return "X";
    }

    setJoystickRobot(id: number) {
        // Toggle the selected robot off if it is the same
        if (this.controlled_robot == id) {
            this.controlled_robot = -1;
        } else {
            this.controlled_robot = id;
        }
        this.params["joystick_param"].set(this.controlled_robot);
    }


    setPlayEnabled(play: Play) {
        const state = this; // fix dumb javascript things
        const request = new ROSLIB.ServiceRequest({
            play_name: play.name,
            enabled: play.enabled
        });

        this.services["setPlayEnabled"].callService(request,
            function(result) {
                if(!result.success) {
                    console.log("Failed to enable/disable ", play.name, ": ", result.reason);
                }
            });
    }

    setOverridePlay(play_name: string) {
        const state = this; // fix dumb javascript things
        const request = new ROSLIB.ServiceRequest({
            play_name: play_name
        });

        state.override_play_in_progress = play_name;
        this.services["setOverridePlay"].callService(request,
            function(result) {
                state.override_play_in_progress = null;
                if(!result.success) {
                    console.log("Failed to set override play to ", play_name, ": ", result.reason);
                }
            });
    }

    setIgnoreFieldSide(ignore_side: number) {
        let int_side = 0;
        if (ignore_side > 0) {
            int_side = 1;
        } else if (ignore_side < 0) {
            int_side = -1;
        }
        const request = new ROSLIB.ServiceRequest({
            ignore_side: int_side
        });

        this.services["setIgnoreFieldSide"].callService(request,
            function(result) {
                if(!result.success) {
                    console.error("Failed to set ignore side: ", result.reason);
                }
            });
    }

    // TODO(chachmu): break this out into better functions for different things
    sendSimulatorControlPacket(simulator_control_packet) {
        const request = new ROSLIB.ServiceRequest({
            simulator_control: simulator_control_packet
        });

        this.services["sendSimulatorControlPacket"].callService(request,
            function(result) {
                if(!result.success) {
                    console.log("Failed to send simulator packet: ", result.reason);
                }
            });
    }

    getTeamNameCallback() {
        const state = this; // fix dumb javascript things
        return function(value: any) {
            state.world.team_name = value;

            if (state.world.referee.blue.name == state.world.team_name) {
                state.world.team = TeamColor.Blue;
            } else if (state.world.referee.yellow.name == state.world.team_name) {
                state.world.team = TeamColor.Yellow;
            }
        }
    }

    // TODO: figure out how to type ROSLIB Messages, the Message type doesn't seem to work properly
    getBallCallback() {
        const state = this; // fix dumb javascript things
        return function(msg: any) {
            state.world.ball.pose = msg.pose;
            state.world.ball.visible = msg.visible;
        }
    }

    getRobotCallback(team: string, id: number) {
	const state = this; // fix dumb javascript things
        return function(msg: any) {
            let robot = state.world.teams[team].robots[id];
            robot.pose = msg.pose;
            robot.twist = msg.twist;
            robot.accel = msg.accel;
            robot.visible = msg.visible;
        };

    }

    getRobotStatusCallback(id: number) {
	const state = this; // fix dumb javascript things
        return function(msg: any) {
            let robot = state.world.teams[state.world.team].robots[id];
            for (const member of Object.getOwnPropertyNames(msg)) {
                robot.status[member] = msg[member];
            }
        };

    }

    getOverlayCallback() {
	    const state = this; // fix dumb javascript things
	    return function(msg:any) {
            for (const overlay of msg.overlays) {
                let id = overlay.ns+"/"+overlay.name;

                // Flag the overlay to check if it moved the graphic object between containers
                const check_other_depth =  (id in state.world.field.overlays &&
                    state.world.field.overlays[id].depth != overlay.depth);

                // Overlays will now be deleted in the update function if
                // the lifetime expires without having been updated
                // to prevent potential future memory leaks
                switch(overlay.command) {
                    // REPLACE
                    case 0:
                        state.world.field.overlays[id] = new Overlay(id, overlay, check_other_depth);
                        break;
                    // EDIT
                    case 1:
                        //TODO: Not sure if this command is necessary, will implement later if it is
                        // Might need to handle moving overlay between z-depths
                        state.world.field.overlays[id] = new Overlay(id, overlay, check_other_depth);
                        break;
                    // REMOVE
                    case 2:
                        // I think this just leaves the graphics objects in the list :'(
                        // TODO: fix memory leak ^
                        delete state.world.field.overlays[id];
                        break;
                }
            }
        }
    }

    getFieldDimensionCallback() {
	    const state = this; // fix dumb javascript things
	    return function(msg:any) {
            state.world.field.fieldDimensions.length = msg.field_length;
            state.world.field.fieldDimensions.width = msg.field_width;
            state.world.field.fieldDimensions.goalWidth = msg.goal_width;
            state.world.field.fieldDimensions.goalDepth = msg.goal_depth;
            state.world.field.fieldDimensions.border = msg.boundary_width;

            let get_dims = function(rectangle_corners:Point[]) {
                const first_point = rectangle_corners[0];

                // find the opposite corner
                let second_point;
                for (let i = 1; i < rectangle_corners.length; i++) {
                    second_point = rectangle_corners[i];
                    if (first_point.x != second_point.x && first_point.y != second_point.y) {
                        break;
                    }
                }

                const dim_1 = Math.abs(first_point.x - second_point.x);
                const dim_2 = Math.abs(first_point.y - second_point.y);

                return [Math.min(dim_1, dim_2), Math.max(dim_1, dim_2)]
            }

            if (msg.ours.defense_area_corners.points.length) {
                const defense_dims = get_dims(msg.ours.defense_area_corners.points);
                state.world.field.fieldDimensions.penaltyShort = defense_dims[0];
                state.world.field.fieldDimensions.penaltyLong = defense_dims[1];
            }

            if (msg.ours.goal_corners.points.length) {
                const goal_dims = get_dims(msg.ours.goal_corners.points);
                state.world.field.fieldDimensions.goalDepth = goal_dims[0];
                state.world.field.fieldDimensions.goalWidth = goal_dims[1];
            }

            state.world.ignore_side = msg.ignore_side;
        }
    }

    getRefereeCallback() {
	    const state = this; // fix dumb javascript things
	    return function(msg:any) {
            // TODO: Check how well this works, the referee class doesn't exactly match Referee.msg type
            for (const member of Object.getOwnPropertyNames(state.world.referee)) {
                state.world.referee[member] = msg[member];
            }

            state.world.teams[TeamColor.Blue].defending = msg.blue_team_on_positive_half ? 1 : -1;
            state.world.teams[TeamColor.Yellow].defending = msg.blue_team_on_positive_half ? -1 : 1;

            // TODO: probably pull this from the ros network instead
            if (state.world.referee.blue.name == state.world.team_name) {
                state.world.team = TeamColor.Blue;
            } else if (state.world.referee.yellow.name == state.world.team_name) {
                state.world.team = TeamColor.Yellow;
            }
        }
    }

    getPlayInfoCallback() {
	    const state = this; // fix dumb javascript things
	    return function(msg:any) {
                state.world.ai.name = msg.name;
                state.world.ai.description = msg.description;
        }
    }

    getPlaybookCallback() {
        const state = this; // fix dumb javascript things
        return function(msg:any) {

            // Handle the transition time between overriding the play and
            // it showing up in the kenobi message
            if (state.override_play_in_progress == null) {
                state.selected_play_name = msg.override_name;
            } else {
                state.selected_play_name = state.override_play_in_progress;
            }


            if (msg.names.length > 0) {
                state.plays = [];
                for (let i = 0; i < msg.names.length; i++) {
                    if (msg.names[i] in state.plays) {
                        state.plays[msg.names[i]].enabled = msg.enableds[i]
                        state.plays[msg.names[i]].score = msg.scores[i]
                    } else {
                        let play = new Play(
                            msg.names[i],
                            msg.enableds[i],
                            msg.scores[i]
                        )

                        state.plays[play.name] = play;
                    }
                }
            }
        }
    }

    getJoystickStatusCallback() {
        const state = this; // fix dumb javascript things
        return function(msg:any) {
            if (msg.is_active) {
                state.controlled_robot = msg.active_id;
            } else {
                state.controlled_robot = -1;
            }
        }
    }

    constructor() {
        this.renderConfig = new RenderConfig();
        this.world = new WorldState();

        this.publishers = [];
        this.subscriptions = [];
        this.services = [];
        this.params = [];

        this.controlled_robot = null;

        // Configure ROS
        this.ros = new ROSLIB.Ros({
            url : 'ws://' + location.hostname + ':9090'
        });

        this.ros.on('connection', function() {
            console.log('Connected to ROS server.');
        });

        this.ros.on('error', function(error) {
            console.error('Error connecting to ROS server: ', error);
        });

        this.ros.on('close', function() {
            console.log('Connection to ROS server closed.');
         
            // TODO: Handle disconnecting from ROS
            //Neutralino.app.exit();
        });

    }

    mount() {
        // TODO: add a way to handle ROS namespaces

        // Set up ball subscribers and publishers
        let ballTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ball',
            messageType: 'ateam_msgs/msg/BallState'
        });

        ballTopic.subscribe(this.getBallCallback());
        this.subscriptions["ball"] = ballTopic;
        //TODO: add publisher for moving sim ball

        for (var i = 0; i < 16; i++) {
            for (var team in this.world.teams) {
                let robotTopic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/' + team + '_team/robot' + i,
                    messageType: 'ateam_msgs/msg/RobotState'
                });

                robotTopic.subscribe(this.getRobotCallback(team, i));
                this.subscriptions['/' + team + '_team/robot' + i] = robotTopic;


                //TODO: add publisher for moving sim robots
            }

            let robotStatusTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/robot_feedback/status/robot' + i,
                messageType: 'ateam_msgs/msg/RobotFeedback'
            });

            robotStatusTopic.subscribe(this.getRobotStatusCallback(i));
            this.subscriptions['/robot_feedback/status/robot' + i] = robotStatusTopic;
        }

        // Set up overlay subscriber
        let overlayTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/overlays',
            messageType: 'ateam_msgs/msg/OverlayArray'
        });
        overlayTopic.subscribe(this.getOverlayCallback());
        this.subscriptions["overlay"] = overlayTopic;

        // Set up fieldDimension subscriber
        let fieldDimensionTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/field',
            messageType: 'ateam_msgs/msg/FieldInfo'
        });

        fieldDimensionTopic.subscribe(this.getFieldDimensionCallback());
        this.subscriptions["fieldDimension"] = fieldDimensionTopic;

        // Set up referee subscriber
        let refereeTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/referee_messages',
            messageType: 'ssl_league_msgs/msg/Referee'
        });

        refereeTopic.subscribe(this.getRefereeCallback());
        this.subscriptions["referee"] = refereeTopic;

        // Set up play info subscriber
        let playInfoTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/play_info',
            messageType: 'ateam_msgs/msg/PlayInfo'
        });

        playInfoTopic.subscribe(this.getPlayInfoCallback());
        this.subscriptions["playInfo"] = playInfoTopic;

        // Set up play book subscriber
        let playbookTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/kenobi_node/playbook_state',
            messageType: 'ateam_msgs/msg/PlaybookState'
        });

        playbookTopic.subscribe(this.getPlaybookCallback());
        this.subscriptions["playbook"] = playbookTopic;

        // Set up joystick control status subscriber
        let joystickStatusTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/joystick_control_status',
            messageType: 'ateam_msgs/msg/JoystickControlStatus'
        });

        joystickStatusTopic.subscribe(this.getJoystickStatusCallback());
        this.subscriptions["joystickStatus"] = joystickStatusTopic;

        // Set up Goalie Service
        let goalieService = new ROSLIB.Service({
            ros: this.ros,
            name: '/team_client_node/set_desired_keeper',
            serviceType: 'ateam_msgs/srv/SetDesiredKeeper'
        })
        this.services["setGoalie"] = goalieService;

        // Set up set play enabled Service
        let setPlayEnabledService = new ROSLIB.Service({
            ros: this.ros,
            name: '/kenobi_node/set_play_enabled',
            serviceType: 'ateam_msgs/srv/SetPlayEnabled'
        })
        this.services["setPlayEnabled"] = setPlayEnabledService;

        // Set up set override play Service
        let setOverridePlayService = new ROSLIB.Service({
            ros: this.ros,
            name: '/kenobi_node/set_override_play',
            serviceType: 'ateam_msgs/srv/SetOverridePlay'
        })
        this.services["setOverridePlay"] = setOverridePlayService;

        let teamNameParam = new ROSLIB.Param({
            ros: this.ros,
            name: "/team_client_node:team_name"
        });
        teamNameParam.get(this.getTeamNameCallback());
        this.params["team_name_param"] = teamNameParam;

        // Set up joystick robot param service
        const joystickParam = new ROSLIB.Param({
            ros: this.ros,
            name: "/joystick_control_node:robot_id"
        });
        this.params["joystick_param"] = joystickParam;

        // Set up set ignore field side service
        let setIgnoreFieldSideService = new ROSLIB.Service({
            ros: this.ros,
            name: '/field_manager/set_ignore_field_side',
            serviceType: 'ateam_msgs/srv/SetIgnoreFieldSide'
        })
        this.services["setIgnoreFieldSide"] = setIgnoreFieldSideService;

        // Set up set ignore field side service
        let sendSimulatorControlPacketService = new ROSLIB.Service({
            ros: this.ros,
            name: '/radio_bridge/send_simulator_control_packet',
            serviceType: 'ateam_msgs/srv/SendSimulatorControlPacket'
        })
        this.services["sendSimulatorControlPacket"] = sendSimulatorControlPacketService;
    }
}
