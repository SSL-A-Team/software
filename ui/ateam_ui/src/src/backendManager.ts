import { TeamColor } from "@/team";
import { Overlay, OverlayType, deleteOverlayGraphic, initializeHeatmapGraphic } from "@/overlay";
import { Play } from "@/play";
import { WorldState, AppState } from "@/state";
import { FieldInfo } from "./field";
import { App } from "vue";

export enum MessageType {
  // Internal UI Backend commands
  SetUseKenobiTopic,
  // Params
  SetJoystickRobot,
  // Services
  SetDesiredKeeper,
  SetPlayEnabled,
  SetOverridePlay,
  SetIgnoreFieldSide,
  SendPowerRequest,
  SendRebootKenobiRequest,
  SendSimulatorControlPacket
}

export class BackendResponse {
    msg_type: number;
    success: boolean;
    reason: string;
}

export class BackendCommand {
    msg_type: number;
}

export class BackendMessage {
    world: WorldState;
    responses: BackendResponse[]
}

export class BackendManager {
    websocket: WebSocket;
    numReceived: number;
    time: number;

    constructor(state: AppState) {

        console.log(state);
        console.log("Connecting to 9001")
        this.websocket = new WebSocket('ws://' + location.hostname + ':9001');


        let numReceived = this.numReceived;
        let time = this.time;
        this.websocket.onopen = function(event) {
            console.log("Connected", event);
            time = Date.now();
            numReceived = 0;
        };

        this.websocket.onmessage = this.getReceiveBackendMessage(state);
        this.websocket.onclose = function(event) {
            console.log("Disconnected from websocket", event);
        };

        return;

        // // Set up topics, services, and params

        // for (var i = 0; i < 16; i++) {
        //     for (const team of appState.realtimeWorld.teams.keys()) {
        //         let robotConnectionTopic = new ROSLIB.Topic({
        //             ros: this.ros,
        //             name: '/robot_feedback/connection/robot' + i,
        //             messageType: 'ateam_radio_msgs/msg/ConnectionStatus'
        //         });

        //         robotConnectionTopic.subscribe(this.getRobotConnectionCallback(appState.realtimeWorld, i));
        //         this.subscriptions.set("/robot_feedback/connection/robot" + i, robotConnectionTopic);

        //         let robotBasicTopic = new ROSLIB.Topic({
        //             ros: this.ros,
        //             name: '/robot_feedback/basic/robot' + i,
        //             messageType: 'ateam_radio_msgs/msg/BasicTelemetry'
        //         });

        //         robotBasicTopic.subscribe(this.getRobotBasicCallback(appState.realtimeWorld, i));
        //         this.subscriptions.set("/robot_feedback/basic/robot" + i, robotBasicTopic);
        //     }
        // }

        // let overlayTopic = new ROSLIB.Topic({
        //     ros: this.ros,
        //     name: '/overlays',
        //     messageType: 'ateam_msgs/msg/OverlayArray'
        // });
        // overlayTopic.subscribe(this.getOverlayCallback(appState));
        // this.subscriptions.set("overlay", overlayTopic);

        // let fieldDimensionTopic = new ROSLIB.Topic({
        //     ros: this.ros,
        //     name: '/field',
        //     messageType: 'ateam_msgs/msg/FieldInfo'
        // });

        // fieldDimensionTopic.subscribe(this.getFieldDimensionCallback(appState.realtimeWorld));
        // this.subscriptions.set("fieldDimension", fieldDimensionTopic);

        // let refereeTopic = new ROSLIB.Topic({
        //     ros: this.ros,
        //     name: '/referee_messages',
        //     messageType: 'ssl_league_msgs/msg/Referee'
        // });

        // refereeTopic.subscribe(this.getRefereeCallback(appState.realtimeWorld));
        // this.subscriptions.set("referee", refereeTopic);

        // let playInfoTopic = new ROSLIB.Topic({
        //     ros: this.ros,
        //     name: '/play_info',
        //     messageType: 'ateam_msgs/msg/PlayInfo'
        // });

        // playInfoTopic.subscribe(this.getPlayInfoCallback(appState.realtimeWorld));
        // this.subscriptions.set("playInfo", playInfoTopic);

        // let playbookTopic = new ROSLIB.Topic({
        //     ros: this.ros,
        //     name: '/kenobi_node/playbook_state',
        //     messageType: 'ateam_msgs/msg/PlaybookState'
        // });

        // playbookTopic.subscribe(this.getPlaybookCallback(appState));
        // this.subscriptions.set("playbook", playbookTopic);

        // let joystickStatusTopic = new ROSLIB.Topic({
        //     ros: this.ros,
        //     name: '/joystick_control_status',
        //     messageType: 'ateam_msgs/msg/JoystickControlStatus'
        // });

        // joystickStatusTopic.subscribe(this.getJoystickStatusCallback(appState));
        // this.subscriptions.set("joystickStatus", joystickStatusTopic);

        // let goalieService = new ROSLIB.Service({
        //     ros: this.ros,
        //     name: '/team_client_node/set_desired_keeper',
        //     serviceType: 'ateam_msgs/srv/SetDesiredKeeper'
        // })
        // this.services.set("setGoalie", goalieService);

        // let setPlayEnabledService = new ROSLIB.Service({
        //     ros: this.ros,
        //     name: '/kenobi_node/set_play_enabled',
        //     serviceType: 'ateam_msgs/srv/SetPlayEnabled'
        // })
        // this.services.set("setPlayEnabled", setPlayEnabledService);

        // let setOverridePlayService = new ROSLIB.Service({
        //     ros: this.ros,
        //     name: '/kenobi_node/set_override_play',
        //     serviceType: 'ateam_msgs/srv/SetOverridePlay'
        // })
        // this.services.set("setOverridePlay", setOverridePlayService);

        // let teamNameParam = new ROSLIB.Param({
        //     ros: this.ros,
        //     name: "/team_client_node:team_name"
        // });
        // teamNameParam.get(this.getTeamNameCallback(appState.realtimeWorld));
        // this.params.set("team_name_param", teamNameParam);

        // const joystickParam = new ROSLIB.Param({
        //     ros: this.ros,
        //     name: "/joystick_control_node:robot_id"
        // });
        // this.params.set("joystick_param", joystickParam);

        // let setIgnoreFieldSideService = new ROSLIB.Service({
        //     ros: this.ros,
        //     name: '/field_manager/set_ignore_field_side',
        //     serviceType: 'ateam_msgs/srv/SetIgnoreFieldSide'
        // })
        // this.services.set("setIgnoreFieldSide", setIgnoreFieldSideService);

        // let sendSimulatorControlPacketService = new ROSLIB.Service({
        //     ros: this.ros,
        //     name: '/radio_bridge/send_simulator_control_packet',
        //     serviceType: 'ateam_msgs/srv/SendSimulatorControlPacket'
        // })
        // this.services.set("sendSimulatorControlPacket", sendSimulatorControlPacketService);

        // let sendPowerRequestService = new ROSLIB.Service({
        //     ros: this.ros,
        //     name: '/radio_bridge/send_power_request',
        //     serviceType: 'ateam_msgs/srv/SendRobotPowerRequest'
        // })
        // this.services.set("sendPowerRequest", sendPowerRequestService);

        // let sendRebootKenobiRequestService = new ROSLIB.Service({
        //     ros: this.ros,
        //     name: '/strike_him_down',
        //     serviceType: 'std_msgs/srv/Trigger'
        // })
        // this.services.set("sendRebootKenobiRequest", sendRebootKenobiRequestService);

        // if (appState.useKenobi) {
        //     this.enableKenobiTopic(appState);
        // } else {
        //     this.enableStateTopics(appState);
        // }
    }

    // enableKenobiTopic(appState: AppState) {
    //     let kenobiTopic = this.subscriptions.get("kenobi");
    //     if (!kenobiTopic) {
    //         kenobiTopic = new ROSLIB.Topic({
    //             ros: this.ros,
    //             name: '/kenobi_node/world',
    //             messageType: 'ateam_msgs/msg/World'
    //         });
    //         this.subscriptions.set("kenobi", kenobiTopic);
    //     }

    //     kenobiTopic.subscribe(this.getKenobiCallback(appState));
    // }

    // disableKenobiTopic(appState: AppState) {
    //     let kenobiTopic = this.subscriptions.get("kenobi");
    //     if (kenobiTopic) {
    //         kenobiTopic.unsubscribe();
    //     }
    // }

    // enableStateTopics(appState: AppState) {
    //     this.websocket.send("COMMAND: enableStateTopics");
    //     console.log("sent enable");
    //     return; // TODO: REMOVE ALL OF THIS

    //     let ballTopic = this.subscriptions.get("ball");
    //     if (!ballTopic) {
    //         ballTopic = new ROSLIB.Topic({
    //             ros: this.ros,
    //             name: '/ball',
    //             messageType: 'ateam_msgs/msg/BallState'
    //         });
    //         this.subscriptions.set("ball", ballTopic);
    //     }
    //     ballTopic.subscribe(this.getBallCallback(appState.realtimeWorld));

    //     for (var i = 0; i < 16; i++) {
    //         for (const team of appState.realtimeWorld.teams.keys()) {
    //             let robotTopic = this.subscriptions.get("/" + team + "_team/robot" + i);
    //             if (!robotTopic) {
    //                 robotTopic = new ROSLIB.Topic({
    //                     ros: this.ros,
    //                     name: '/' + team + '_team/robot' + i,
    //                     messageType: 'ateam_msgs/msg/RobotState'
    //                 });
    //                 this.subscriptions.set("/" + team + "_team/robot" + i, robotTopic);
    //             }

    //             robotTopic.subscribe(this.getRobotCallback(appState.realtimeWorld, team, i));
    //         }
    //     }
    // }

    // disableStateTopics(appState: AppState) {
    //     this.websocket.send("COMMAND: disableStateTopics");
    //     console.log("sent disable");
    //     return; // TODO: REMOVE ALL OF THIS

    //     for (var i = 0; i < 16; i++) {
    //         for (const team of appState.realtimeWorld.teams.keys()) {
    //             let robotTopic = this.subscriptions.get("/" + team + "_team/robot" + i);
    //             if (robotTopic)  {
    //                 robotTopic.unsubscribe();
    //             }
    //         }
    //     }

    //     let ballTopic = this.subscriptions.get("ball");
    //     if (ballTopic) {
    //         ballTopic.unsubscribe();
    //     }
    // }

    getReceiveBackendMessage(state: AppState) {
        const backendManager = this;
        let world = state.realtimeWorld;

        const worldUpdate = this.getKenobiCallback(state);

        return function (event: MessageEvent) {
            backendManager.numReceived++;
            // const msg: BackendMessage = JSON.parse(event.data);
            const msg = JSON.parse(event.data);
            console.log(msg);
            worldUpdate(msg.world);
            state.realtimeWorld.fps = 1000 * backendManager.numReceived / (Date.now() - backendManager.time);

            if (msg.responses.length > 0) {
                console.log("responses["+ msg.responses.length+ "]:");
                for (const result of msg.responses) {
                    console.log(result);
                    switch (result.msg_type) {
                        case MessageType.SetDesiredKeeper:
                            backendManager.handleSetKeeperResponse(state, result);
                        default:
                            console.log("message type not recognized")
                    }
                }
            }
        }
    }

    // TODO: It might be nice to be able to have this trigger a callback
    sendBackendCommand(command: any) {
        this.websocket.send(JSON.stringify(command));
    }

    handleSetKeeperResponse(state: AppState, result: BackendResponse) {
        if(!result.success) {
            state.goalieServiceStatus = [false, result.reason];
        } else {
            state.goalieServiceStatus = [true, ""];
        }
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
            // Convert timestamp to millis
            appState.realtimeWorld.timestamp = (msg.timestamp.sec * 1e3) + (msg.timestamp.nanosec / 1e6);
            appState.realtimeWorld.fps = msg.fps;
            appState.lastTimeReceivedKenobi = Date.now();

            if (msg.balls.length > 0) {
                appState.realtimeWorld.ball.pose = msg.balls[0].pose;
                appState.realtimeWorld.ball.visible = msg.balls[0].visible;
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

            if (appState.useKenobi) {
                appState.updateHistory();
            } else {
                // This should never happen
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

    getRobotConnectionCallback(world: WorldState, id: number): (msg: any) => void {
        return function(msg: any): void {
            let robot = world.teams.get(world.team).robots[id];
            robot.radio_connected = msg.radio_connected;
        };
    }

    getRobotBasicCallback(world: WorldState, id: number): (msg: any) => void {
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
                const checkOtherDepth = (id in appState.realtimeWorld.field.overlays &&
                    appState.realtimeWorld.field.overlays.get(id).depth != overlay.depth);

                switch(overlay.command) {
                    // REPLACE
                    case 0:
                        appState.realtimeWorld.field.overlays.set(id, new Overlay(id, overlay, checkOtherDepth, appState.realtimeWorld.timestamp));
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

            let field_info = world.field;
            for (const member of Object.getOwnPropertyNames(msg)) {
                field_info[member] = msg[member];
            }

            // Don't accidentally trigger field dimension update callbacks in vue
            // let newFieldDimensions = new FieldInfo();

            // newFieldDimensions.field_length = msg.field_length;
            // newFieldDimensions.field_width = msg.field_width;
            // newFieldDimensions.goal_width = msg.goal_width;
            // newFieldDimensions.goal_depth = msg.goal_depth;
            // newFieldDimensions.boundary_width = msg.boundary_width;

            // let getDims = function(rectangleCorners: Point[]): [number, number] {
            //     const firstPoint = rectangleCorners[0];

            //     // find the opposite corner
            //     let secondPoint: Point;
            //     for (let i = 1; i < rectangleCorners.length; i++) {
            //         secondPoint = rectangleCorners[i];
            //         if (firstPoint.x != secondPoint.x && firstPoint.y != secondPoint.y) {
            //             break;
            //         }
            //     }

            //     const dim1 = Math.abs(firstPoint.x - secondPoint.x);
            //     const dim2 = Math.abs(firstPoint.y - secondPoint.y);

            //     return [Math.min(dim1, dim2), Math.max(dim1, dim2)]
            // }

            // if (msg.ours.defense_area_corners.points.length) {
            //     const defenseDims = getDims(msg.ours.defense_area_corners.points);
            //     newFieldDimensions.defense_area_depth = defenseDims[0];
            //     newFieldDimensions.defense_area_width = defenseDims[1];
            // }

            // if (msg.ours.goal_corners.points.length) {
            //     const goalDims = getDims(msg.ours.goal_corners.points);
            //     newFieldDimensions.goal_depth = goalDims[0];
            //     newFieldDimensions.goal_width = goalDims[1];
            // }

            // world.field.fieldInfo = newFieldDimensions;
            // world.ignore_side = msg.ignore_side;
        }
    }

    getRefereeCallback(world: WorldState): (msg: any) => void {
	    return function(msg: any): void {
            world.referee.stage = msg.stage;
            world.referee.stage_time_left = msg.stage_time_left;
            world.referee.command = msg.command;
            world.referee.command_counter = msg.command_counter;
            world.referee.command_timestamp = msg.command_timestamp;

            world.referee.yellow = msg.yellow;
            world.referee.blue = msg.blue;

            world.referee.designatedPosition = msg.designatedPosition;
            world.referee.next_command = msg.next_command;
            world.referee.current_action_time_remaining = msg.current_action_time_remaining;

            if (msg.blue_team_on_positive_half.length != 0) {
                world.teams.get(TeamColor.Blue).defending = msg.blue_team_on_positive_half[0] ? 1 : -1;
                world.teams.get(TeamColor.Yellow).defending = msg.blue_team_on_positive_half[0] ? -1 : 1;
            }

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
