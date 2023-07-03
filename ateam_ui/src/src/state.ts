import ROSLIB from "roslib"
// import 'roslib/build/roslib';
import { Team, TeamInfo, TeamColor } from "@/team"
import { Overlay } from "@/overlay"
import { Referee } from "@/referee"
import { Ball } from "@/ball"
import { Field } from "@/field"

export class RenderConfig {
    angle: number = 0; // Rotation applied to the rendered field
    scale: number = 140; // Pixels per meter (in the rendering canvas)   
}

export class WorldState {
    team: TeamColor;
    teams: Team[];
    ball: Ball;
    field: Field;
    referee: Referee

    constructor() {
        this.team = TeamColor.Blue;
        this.teams = [];
        this.teams[TeamColor.Blue] = new Team("A-Team", TeamColor.Blue, -1);
        this.teams[TeamColor.Yellow] = new Team("Opponent", TeamColor.Yellow, 1);

        this.ball = new Ball();
        this.field = new Field();

        this.referee = new Referee();
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
    param: ROSLIB.Param[]

    sim: boolean = true;
    comp: boolean = true; // TODO: find a better way to set this value

    controlled_robot: number = null;

    setGoalie(goalie_id) {
        const request = new ROSLIB.ServiceRequest({
            desired_keeper: goalie_id
        });

        this.services["setGoalie"].callService(request,
            function(result) {
                // is there anything we need to do with this?
            });
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

        this.params["joystick_param"].set(id);
    }

    // TODO: figure out how to type ROSLIB Messages, the Message type doesn't seem to work properly
    getBallCallback() {
        const state = this; // fix dumb javascript things
        return function(msg: any) {
            state.world.ball.pose = msg.pose;
        }
    }

    getRobotCallback(team: string, id: number) {
	const state = this; // fix dumb javascript things
        return function(msg: any) {
            let robot = state.world.teams[team].robots[id];
            robot.pose = msg.pose;
            robot.twist = msg.twist;
            robot.accel = msg.accel;
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
            let id = msg.ns+"/"+msg.name;
            switch(msg.command) {
                // REPLACE
                case 0:
                    state.world.field.overlays[id] = new Overlay(id, msg);
                    break;
                // EDIT
                case 1:
                    //TODO: Not sure if this command is necessary, will implement later if it is
                    // Might need to handle moving overlay between z-depths
                    state.world.field.overlays[id] = new Overlay(id, msg);
                    break;
                // REMOVE
                case 2:
                    delete state.world.field.overlays[id];
                    break;
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
        }
    }

    getRefereeCallback() {
	    const state = this; // fix dumb javascript things
	    return function(msg:any) {
            // TODO: Check how well this works, the referee class doesn't exactly match Referee.msg type
            for (const member of Object.getOwnPropertyNames(state.world.referee)) {
                state.world.referee[member] = msg[member];
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
        this.dribbler_speed = 0;

        // Configure ROS
        this.ros = new ROSLIB.Ros({
            url : 'ws://' + location.hostname + ':9090'
        });

        this.ros.on('connection', function() {
            console.log('Connected to ROS server.');
        });

        this.ros.on('error', function(error) {
            console.log('Error connecting to ROS server: ', error);
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
                name: '/robot_feedback/robot' + i,
                messageType: 'ateam_msgs/msg/RobotFeedback'
            });

            robotStatusTopic.subscribe(this.getRobotStatusCallback(i));
            this.subscriptions['/robot_feedback/robot' + i] = robotStatusTopic;
        }

        // Set up overlay subscriber
        let overlayTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/overlay',
            messageType: 'ateam_msgs/msg/Overlay'
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

        // Set up Goalie Service
        let goalieService = new ROSLIB.Service({
            ros: this.ros,
            name: 'team_client_node/set_desired_keeper',
            serviceType: 'ateam_msgs/srv/SetDesiredKeeper'
        })
        this.services["setGoalie"] = goalieService;

        // Set up joystick robot param service
        const joystickParam = new ROSLIB.Param({
            ros: this.ros,
            name: "/joystick_control_node:robot_id"
        });
        this.params["joystick_param"] = joystickParam;
    }
}
