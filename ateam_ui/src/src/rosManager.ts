import ROSLIB from "roslib";
import { TeamColor } from "@/team";
import { Overlay, OverlayType, deleteOverlayGraphic, initializeHeatmapGraphic } from "@/overlay";
import { Play } from "@/play";
import { WorldState, AppState } from "@/state";

export class RosManager {
    ros: ROSLIB.Ros;
    publishers: Map<string, ROSLIB.Topic> = new Map<string, ROSLIB.Topic>;
    subscriptions: Map<string, ROSLIB.Topic> = new Map<string, ROSLIB.Topic>;
    services: Map<string, ROSLIB.Service> = new Map<string, ROSLIB.Service>;
    params: Map<string, ROSLIB.Param> = new Map<string, ROSLIB.Param>;

    constructor(appState: AppState) {

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
        });

        // Set up topics, services, and params

        // let kenobiTopic = new ROSLIB.Topic({
        //     ros: this.ros,
        //     name: '/kenobi_node/world',
        //     messageType: 'ateam_msgs/msg/World'
        // });

        // kenobiTopic.subscribe(this.getKenobiCallback(appState));
        // this.subscriptions.set("kenobi", kenobiTopic);

        let ballTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ball',
            messageType: 'ateam_msgs/msg/BallState'
        });

        ballTopic.subscribe(this.getBallCallback(appState.realtimeWorld));
        this.subscriptions.set("ball", ballTopic);

        // Robot topics for both teams
        for (var i = 0; i < 16; i++) {
            for (const team of appState.realtimeWorld.teams.keys()) {
                let robotTopic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/' + team + '_team/robot' + i,
                    messageType: 'ateam_msgs/msg/RobotState'
                });

                robotTopic.subscribe(this.getRobotCallback(appState.realtimeWorld, team, i));
                this.subscriptions.set("/" + team + "_team/robot" + i, robotTopic);
            }

            let robotStatusTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/robot_feedback/status/robot' + i,
                messageType: 'ateam_msgs/msg/RobotFeedback'
            });

            robotStatusTopic.subscribe(this.getRobotStatusCallback(appState.realtimeWorld, i));
            this.subscriptions.set("/robot_feedback/status/robot" + i, robotStatusTopic);
        }

        let overlayTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/overlays',
            messageType: 'ateam_msgs/msg/OverlayArray'
        });
        overlayTopic.subscribe(this.getOverlayCallback(appState));
        this.subscriptions.set("overlay", overlayTopic);

        let fieldDimensionTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/field',
            messageType: 'ateam_msgs/msg/FieldInfo'
        });

        fieldDimensionTopic.subscribe(this.getFieldDimensionCallback(appState.realtimeWorld));
        this.subscriptions.set("fieldDimension", fieldDimensionTopic);

        let refereeTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/referee_messages',
            messageType: 'ssl_league_msgs/msg/Referee'
        });

        refereeTopic.subscribe(this.getRefereeCallback(appState.realtimeWorld));
        this.subscriptions.set("referee", refereeTopic);

        let playInfoTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/play_info',
            messageType: 'ateam_msgs/msg/PlayInfo'
        });

        playInfoTopic.subscribe(this.getPlayInfoCallback(appState.realtimeWorld));
        this.subscriptions.set("playInfo", playInfoTopic);

        let playbookTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/kenobi_node/playbook_state',
            messageType: 'ateam_msgs/msg/PlaybookState'
        });

        playbookTopic.subscribe(this.getPlaybookCallback(appState));
        this.subscriptions.set("playbook", playbookTopic);

        let joystickStatusTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/joystick_control_status',
            messageType: 'ateam_msgs/msg/JoystickControlStatus'
        });

        joystickStatusTopic.subscribe(this.getJoystickStatusCallback(appState));
        this.subscriptions.set("joystickStatus", joystickStatusTopic);

        let goalieService = new ROSLIB.Service({
            ros: this.ros,
            name: '/team_client_node/set_desired_keeper',
            serviceType: 'ateam_msgs/srv/SetDesiredKeeper'
        })
        this.services.set("setGoalie", goalieService);

        let setPlayEnabledService = new ROSLIB.Service({
            ros: this.ros,
            name: '/kenobi_node/set_play_enabled',
            serviceType: 'ateam_msgs/srv/SetPlayEnabled'
        })
        this.services.set("setPlayEnabled", setPlayEnabledService);

        let setOverridePlayService = new ROSLIB.Service({
            ros: this.ros,
            name: '/kenobi_node/set_override_play',
            serviceType: 'ateam_msgs/srv/SetOverridePlay'
        })
        this.services.set("setOverridePlay", setOverridePlayService);

        let teamNameParam = new ROSLIB.Param({
            ros: this.ros,
            name: "/team_client_node:team_name"
        });
        teamNameParam.get(this.getTeamNameCallback(appState.realtimeWorld));
        this.params["team_name_param"] = teamNameParam;

        const joystickParam = new ROSLIB.Param({
            ros: this.ros,
            name: "/joystick_control_node:robot_id"
        });
        this.params.set("joystick_param", joystickParam);

        let setIgnoreFieldSideService = new ROSLIB.Service({
            ros: this.ros,
            name: '/field_manager/set_ignore_field_side',
            serviceType: 'ateam_msgs/srv/SetIgnoreFieldSide'
        })
        this.services.set("setIgnoreFieldSide", setIgnoreFieldSideService);

        let sendSimulatorControlPacketService = new ROSLIB.Service({
            ros: this.ros,
            name: '/radio_bridge/send_simulator_control_packet',
            serviceType: 'ateam_msgs/srv/SendSimulatorControlPacket'
        })
        this.services.set("sendSimulatorControlPacket", sendSimulatorControlPacketService);
    }

    getTeamNameCallback(world: WorldState): (value: any) => void {
        return function(value: any): void {
            world.teamName = value;

            if (world.referee.blue.name == world.teamName) {
                world.team = TeamColor.Blue;
            } else if (world.referee.yellow.name == world.teamName) {
                world.team = TeamColor.Yellow;
            }
        }
    }

    getKenobiCallback(appState: AppState): (msg: any) => void {
        return function(msg: any): void {
            appState.realtimeWorld.timestamp = Date.now();

            if (msg.balls.length > 0) {
                appState.realtimeWorld.ball.pose = msg.balls[0].pose;
                // appState.realtimeWorld.ball.visible = msg.balls[0].visible;
                appState.realtimeWorld.ball.visible = true;
            }

            for (const team of [TeamColor.Blue, TeamColor.Yellow]) {
                const msgRobotArray = (team === appState.realtimeWorld.team) ? msg.our_robots : msg.their_robots;

                for (let id = 0; id < msgRobotArray.length; id++) {
                    appState.realtimeWorld.teams.get(team).robots[id].pose = msgRobotArray[id].pose;
                    appState.realtimeWorld.teams.get(team).robots[id].twist = msgRobotArray[id].twist;
                    appState.realtimeWorld.teams.get(team).robots[id].accel = msgRobotArray[id].accel;
                    appState.realtimeWorld.teams.get(team).robots[id].visible = msgRobotArray[id].visible;
                }
            }

            // Only store history while we are unpausued
            if (appState.selectedHistoryFrame == -1) {
                if (appState.worldHistory.length < 100000) {
                    // @ts-ignore
                    appState.worldHistory.push(structuredClone(appState.realtimeWorld.__v_raw));
                } else {
                    appState.historyEndIndex++;
                    if (appState.historyEndIndex >= appState.worldHistory.length) {
                        appState.historyEndIndex = 0;
                    }
                    // @ts-ignore
                    appState.worldHistory[appState.historyEndIndex] = structuredClone(appState.realtimeWorld.__v_raw);
                }
            }
        }
    }

    getBallCallback(world: WorldState): (msg: any) => void {
        return function(msg: any): void {
            world.ball.pose = msg.pose;
            world.ball.visible = msg.visible;
        }
    }

    getRobotCallback(world: WorldState, team: TeamColor, id: number): (msg: any) => void {
        return function(msg: any): void {
            let robot = world.teams.get(team).robots[id];
            robot.pose = msg.pose;
            robot.twist = msg.twist;
            robot.accel = msg.accel;
            robot.visible = msg.visible;
        };
    }

    getRobotStatusCallback(world: WorldState, id: number): (msg: any) => void {
        return function(msg: any): void {
            let robot = world.teams.get(world.team).robots[id];
            for (const member of Object.getOwnPropertyNames(msg)) {
                robot.status[member] = msg[member];
            }
        };
    }

    // TODO: this needs further refactoring
    getOverlayCallback(appState: AppState): (msg: any) => void {
	    return function(msg: any): void {
            const underlayContainer = appState.graphicState.underlayContainer;
            const overlayContainer = appState.graphicState.overlayContainer;
            for (const overlay of msg.overlays) {
                let id = overlay.ns+"/"+overlay.name;

                // Flag the overlay to check if it moved the graphic object between containers
                const check_other_depth = (id in appState.realtimeWorld.field.overlays &&
                    appState.realtimeWorld.field.overlays.get(id).depth != overlay.depth);

                switch(overlay.command) {
                    // REPLACE
                    case 0:
                        appState.realtimeWorld.field.overlays.set(id, new Overlay(id, overlay, check_other_depth));
                        if (overlay.type == OverlayType.Heatmap) {
                            initializeHeatmapGraphic(appState.realtimeWorld.field.overlays.get(id), appState.graphicState)
                        }
                        break;
                    // EDIT
                    case 1:
                        //TODO: Not sure if this command is necessary, will implement later if it is
                        break;
                    // REMOVE
                    case 2:
                        deleteOverlayGraphic(appState.realtimeWorld.field.overlays.get(id), underlayContainer);
                        deleteOverlayGraphic(appState.realtimeWorld.field.overlays.get(id), overlayContainer);
                        appState.realtimeWorld.field.overlays.delete(id);
                        break;
                }
            }
        }
    }

    getFieldDimensionCallback(world: WorldState): (msg: any) => void {
	    const state = this; // fix dumb javascript things
	    return function(msg: any): void {
            world.field.fieldDimensions.length = msg.field_length;
            world.field.fieldDimensions.width = msg.field_width;
            world.field.fieldDimensions.goalWidth = msg.goal_width;
            world.field.fieldDimensions.goalDepth = msg.goal_depth;
            world.field.fieldDimensions.border = msg.boundary_width;

            let get_dims = function(rectangle_corners: Point[]): [number, number] {
                const first_point = rectangle_corners[0];

                // find the opposite corner
                let second_point: Point;
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
                world.field.fieldDimensions.penaltyShort = defense_dims[0];
                world.field.fieldDimensions.penaltyLong = defense_dims[1];
            }

            if (msg.ours.goal_corners.points.length) {
                const goal_dims = get_dims(msg.ours.goal_corners.points);
                world.field.fieldDimensions.goalDepth = goal_dims[0];
                world.field.fieldDimensions.goalWidth = goal_dims[1];
            }

            world.ignoreSide = msg.ignore_side;
        }
    }

    getRefereeCallback(world: WorldState): (msg: any) => void {
	    return function(msg: any): void {
            for (const member of Object.getOwnPropertyNames(world.referee)) {
                world.referee[member] = msg[member];
            }

            world.teams.get(TeamColor.Blue).defending = msg.blue_team_on_positive_half ? 1 : -1;
            world.teams.get(TeamColor.Yellow).defending = msg.blue_team_on_positive_half ? -1 : 1;

            if (world.referee.blue.name == world.teamName) {
                world.team = TeamColor.Blue;
            } else if (world.referee.yellow.name == world.teamName) {
                world.team = TeamColor.Yellow;
            }
        }
    }

    getPlayInfoCallback(world: WorldState): (msg: any) => void {
	    return function(msg: any): void {
                world.ai.name = msg.name;
                world.ai.description = msg.description;
        }
    }

    getPlaybookCallback(appState: AppState): (msg: any) => void {
        return function(msg: any): void {

            // Handle the transition time between overriding the play and
            // it showing up in the kenobi message
            if (appState.overridePlayInProgress == null) {
                appState.selectedPlayName = msg.override_name;
            } else {
                appState.selectedPlayName = appState.overridePlayInProgress;
            }

            if (msg.names.length > 0) {
                appState.plays.clear();
                for (let i = 0; i < msg.names.length; i++) {
                    if (msg.names[i] in appState.plays) {
                        appState.plays.get(msg.names[i]).enabled = msg.enableds[i]
                        appState.plays.get(msg.names[i]).score = msg.scores[i]
                    } else {
                        let play = new Play(
                            msg.names[i],
                            msg.enableds[i],
                            msg.scores[i]
                        )

                        appState.plays.set(play.name, play);
                    }
                }
            }
        }
    }

    getJoystickStatusCallback(appState: AppState): (msg: any) => void {
        return function(msg: any): void {
            if (msg.is_active) {
                appState.controlledRobot = msg.active_id;
            } else {
                appState.controlledRobot = -1;
            }
        }
    }
}
