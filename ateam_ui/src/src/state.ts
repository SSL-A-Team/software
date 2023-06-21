import ROSLIB from "roslib"
// import 'roslib/build/roslib';
import { Team, TeamColor } from "@/team"
import { Ball } from "@/ball"
import { Field } from "@/field"

export class WorldState {
    team: TeamColor;
    teams: Team[];
    ball: Ball;
    field: Field;

    // TODO: Move these into a ref class once we have a full ref ui element
    refStage: string;
    refCommand: string;

    constructor() {
        this.team = TeamColor.Yellow;
        this.teams = [];
        this.teams[TeamColor.Blue] = new Team("A-Team", TeamColor.Blue, -1);
        this.teams[TeamColor.Yellow] = new Team("Opponent", TeamColor.Yellow, 1);

        this.ball = new Ball();

        this.refStage = "First Half";
        this.refCommand = "HALT";

        this.field = new Field();
    }
}

export class AppState {
    world: WorldState;
    history: WorldState[];
    
    // I'm not sure if we will need ongoing access to these
    ros: ROSLIB.Ros;
    ballTopic: ROSLIB.Topic;
    robotTopics: ROSLIB.Topic[];
    overlayTopic: ROSLIB.Topic;

    sim: boolean = true;

    // TODO: figure out how to type ROSLIB Messages, the Message type doesn't seem to work properly
    ballCallback(msg: any) {
        this.world.ball.pose = msg.pose;
    }

    getRobotCallback(team: string, id: number) {
        return function(msg: any) {
            let robot = this.world.teams[team].robots[id];
            robot.pose = msg.pose;
            robot.twist = msg.twist;
            robot.accel = msg.accel;
        };

    }

    overlayCallback(msg: any) {
            let id = msg.ns+"/"+msg.name;
            switch(msg.command) {
                // REPLACE
                case 0:
                    this.world.field.overlays[id] = msg;
                    break;
                // EDIT
                case 1:
                    //TODO: Not sure if this command is necessary, will implement later if it is
                    // Might need to handle moving overlay between z-depths
                    this.world.field.overlays[id] = msg;
                    break;
                // REMOVE
                case 2:
                    delete this.world.field.overlays[id];
                    break;
            }

    }

    constructor() {
        this.world = new WorldState();
        this.history = [];

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

        // Set up ball subscribers and publishers
        this.ballTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ball',
            messageType: 'ateam_msgs/msg/BallState'
        });

        this.ballTopic.subscribe(this.ballCallback);
        //TODO: add publisher for moving sim ball


        for (var team in this.world.teams) {
            for (var i = 0; i < 16; i++) {
                let robotTopic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/' + team + '_team/robot' + i,
                    messageType: 'ateam_msgs/msg/RobotState'
                });

                robotTopic.subscribe(this.getRobotCallback(team, i));

                //TODO: add subscriber for robot status
                //TODO: add publisher for moving sim robots
            }
        }

        // Set up overlay subscriber
        var overlayTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/overlay',
            messageType: 'ateam_msgs/msg/Overlay'
        });

        overlayTopic.subscribe(this.overlayCallback);

        //TODO: add all other pub/subs (field dimensions, referee, etc)
    }
}
