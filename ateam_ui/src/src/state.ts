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

    constructor() {
        this.team = TeamColor.Blue;
        this.teams = [];
        this.teams[TeamColor.Blue] = new Team("A-Team", TeamColor.Blue, -1);
        this.teams[TeamColor.Yellow] = new Team("Opponent", TeamColor.Yellow, 1);

        this.ball = new Ball();
        this.field = new Field();
    }
}

export class AppState {
    renderConfig: RenderConfig;
    world: WorldState;
    history: WorldState[];

    teamInfo: TeamInfo[];

    // I'm not sure if we will need ongoing access to these
    ros: ROSLIB.Ros;
    topics: ROSLIB.Topic[]

    sim: boolean = true;

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
            for (const member of Object.getOwnPropertyNames(robot.status)) {
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
            for (const member of Object.getOwnPropertyNames(state.referee)) {
                state.referee[member] = msg[member];
            }
        }
    }

    constructor() {
        this.renderConfig = new RenderConfig();
        this.world = new WorldState();
        this.history = [];
        this.teamInfo = [];
        this.topics = [];
        this.referee = new Referee();

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

        // TODO: add a way to handle ROS namespaces

        // Set up ball subscribers and publishers
        let ballTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ball',
            messageType: 'ateam_msgs/msg/BallState'
        });

        ballTopic.subscribe(this.getBallCallback());
        this.topics["ball"] = ballTopic;
        //TODO: add publisher for moving sim ball

        for (var i = 0; i < 16; i++) {
            for (var team in this.world.teams) {
                let robotTopic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/' + team + '_team/robot' + i,
                    messageType: 'ateam_msgs/msg/RobotState'
                });

                robotTopic.subscribe(this.getRobotCallback(team, i));
                this.topics['/' + team + '_team/robot' + i] = robotTopic;


                //TODO: add publisher for moving sim robots
            }

            let robotStatusTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/robot_feedback/robot' + i,
                messageType: 'ateam_msgs/msg/RobotFeedback'
            });

            robotStatusTopic.subscribe(this.getRobotStatusCallback(i));
            this.topics['/robot_feedback/robot' + i] = robotStatusTopic;
        }

        // Set up overlay subscriber
        let overlayTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/overlay',
            messageType: 'ateam_msgs/msg/Overlay'
        });

        overlayTopic.subscribe(this.getOverlayCallback());
        this.topics["overlay"] = overlayTopic;

        /* // Need to merge master to get this
        // Set up fieldDimension subscriber
        let fieldDimensionTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/field',
            messageType: 'ateam_msgs/msg/FieldInfo'
        });

        fieldDimensionTopic.subscribe(this.getFieldDimensionCallback());
        this.topics["fieldDimension"] = fieldDimensionTopic;
        */

        // Set up referee subscriber
        let refereeTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/referee_messages',
            messageType: 'ssl_league_msgs/msg/Referee'
        });

        refereeTopic.subscribe(this.getRefereeCallback());
        this.topics["referee"] = refereeTopic;
    }
}
