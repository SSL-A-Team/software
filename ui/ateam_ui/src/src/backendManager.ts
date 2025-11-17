import { TeamColor } from "@/team";
import { Overlay, OverlayType, isOverlayExpired, deleteOverlayGraphic, initializeHeatmapGraphic } from "@/overlay";
import { Robot } from "@/robot";
import { Play } from "@/play";
import { WorldState, AppState } from "@/state";
import { FieldInfo } from "./field";
import { BlobReadable } from "@mcap/browser";
import { McapIndexedReader, McapTypes } from "@mcap/core";
import { parse as parseMsgDefinition } from "@foxglove/rosmsg";
import { parseRos2idl as parseIdlDefinition } from "@foxglove/ros2idl-parser";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import { BSON } from "bson";
import { GameCommandProperties, getCommandProperty, RefereeHistory } from "@/referee";
import { compressedToRobotArray } from "@/history"

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
    backend_receive_function
    constructor(state: AppState) {

        console.log(state);
        console.log("Connecting to 9001")
        this.backend_receive_function = this.getReceiveBackendMessage(state);

        this.connect_websocket(this);

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

    connect_websocket(backend_manager) {
        console.log("trying to connect");
        backend_manager.websocket = new WebSocket('ws://' + location.hostname + ':9001');
        backend_manager.websocket.binaryType = 'arraybuffer'; 

        backend_manager.websocket.onopen = function(event) {
            console.log("Connected", event);
        };

        backend_manager.websocket.onmessage = backend_manager.backend_receive_function;
        backend_manager.websocket.onclose = function(event) {
            console.log("Disconnected from websocket", event);
            // setTimeout(
            //     function() {
            //         backend_manager.connect_websocket(backend_manager);
            //     }, 
            //     2000
            // );
        };
    }

    getReceiveBackendMessage(state: AppState) {
        if (state.historyManager.viewingBag) {
            return;
        }

        const backendManager = this;
        let world = state.realtimeWorld;

        const worldUpdate = this.getKenobiCallback(state);
        const fieldUpdate = this.getFieldDimensionCallback(state.realtimeWorld);
        const playbookUpdate = this.getPlaybookCallback(state);
        const playInfoUpdate = this.getPlayInfoCallback(state.realtimeWorld);
        const overlayUpdate = this.getOverlayCallback(state);
        const refereeUpdate = this.getRefereeCallback(state.realtimeWorld);

        return function (event: MessageEvent) {
            // const msg = JSON.parse(event.data);
            const msg = BSON.deserialize(event.data);

            if (msg.playbook != null) {
                playbookUpdate(msg.playbook);
            }
            if (msg.referee != null) {
                refereeUpdate(msg.referee);
            }
            if (msg.play_info != null) {
                playInfoUpdate(msg.play_info);
            }

            for (const overlay of msg.overlays) {
                overlayUpdate(overlay);
            }

            if (msg.field != null) {
                fieldUpdate(msg.field);
            }

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

            // Handle this last so we can update history if it is ready
            if (msg.world != null) {
                worldUpdate(msg.world);
                state.lastTimeReceivedKenobi = Date.now();
                state.updateHistory();
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

            if (msg.balls.length > 0) {
                appState.realtimeWorld.ball.pose = msg.balls[0].pose;
                appState.realtimeWorld.ball.visible = msg.balls[0].visible;
            }

            // TODO: architect this better
            let robot_packet_length = 9;
            appState.realtimeWorld.teams.get(TeamColor.Blue).robots = compressedToRobotArray(TeamColor.Blue, 
                msg.robots.slice(0,16 * robot_packet_length));
            appState.realtimeWorld.teams.get(TeamColor.Yellow).robots = compressedToRobotArray(TeamColor.Yellow,
                msg.robots.slice(16 * robot_packet_length, 32 * robot_packet_length));

            // for (const team of [TeamColor.Blue, TeamColor.Yellow]) {
            //     const msgRobotArray = (team === appState.realtimeWorld.team) ? msg.our_robots : msg.their_robots;

            //     for (let id = 0; id < msgRobotArray.length; id++) {
            //         appState.realtimeWorld.teams.get(team).robots[id].pose = msgRobotArray[id].pose;
            //         // appState.realtimeWorld.teams.get(team).robots[id].twist = msgRobotArray[id].twist;
            //         // appState.realtimeWorld.teams.get(team).robots[id].accel = msgRobotArray[id].accel;
            //         appState.realtimeWorld.teams.get(team).robots[id].visible = msgRobotArray[id].visible;
            //     }
            // }

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
            let robot = world.teams.get(team).robots[id] as Robot;
            robot.pose = msg.pose;
            robot.twist = msg.twist;
            robot.accel = msg.accel;
            robot.visible = msg.visible;
        };
    }

    getRobotConnectionCallback(world: WorldState, id: number): (msg: any) => void {
        return function(msg: any): void {
            let robot = world.teams.get(world.team).robots[id] as Robot;
            robot.radio_connected = msg.radio_connected;
        };
    }

    getRobotBasicCallback(world: WorldState, id: number): (msg: any) => void {
        return function(msg: any): void {
            let robot = world.teams.get(world.team).robots[id] as Robot;

            for (const member of Object.getOwnPropertyNames(msg)) {
                robot.status[member] = msg[member];
            }
        };
    }

    // TODO: this needs further refactoring
    getOverlayCallback(appState: AppState): (msg: any) => void {
	    return function(msg: any): void {
            let overlayMap = appState.realtimeWorld.field.overlays as Map<string, Overlay>;
            const underlayContainer = appState.graphicState.underlayContainer;
            const overlayContainer = appState.graphicState.overlayContainer;
            for (const overlay of msg.overlays) {
                let id = overlay.ns+"/"+overlay.name;

                // Flag the overlay to check if it moved the graphic object between containers
                const checkOtherDepth = (id in appState.realtimeWorld.field.overlays &&
                    overlayMap.get(id).depth != overlay.depth);

                switch(overlay.command) {
                    // REPLACE
                    case 0:
                        (appState.realtimeWorld.field.overlays as Map<string, Overlay>).set(id, new Overlay(id, overlay, checkOtherDepth, appState.realtimeWorld.timestamp));
                        if (overlay.type == OverlayType.Heatmap) {
                            initializeHeatmapGraphic(overlayMap.get(id), appState.graphicState)
                        }
                        break;
                    // EDIT
                    case 1:
                        //TODO: Not sure if this command is necessary, will implement later if it is
                        break;
                    // REMOVE
                    case 2:
                        deleteOverlayGraphic(overlayMap.get(id), underlayContainer);
                        deleteOverlayGraphic(overlayMap.get(id), overlayContainer);
                        overlayMap.delete(id);
                        break;
                }
            }
        }
    }

    getFieldDimensionCallback(world: WorldState): (msg: any) => void {
	    const state = this; // fix dumb javascript things
	    return function(msg: any): void {

            let newFieldInfo = new FieldInfo();
            for (const member of Object.getOwnPropertyNames(msg)) {
                newFieldInfo[member] = msg[member];
            }
            world.field.fieldInfo = newFieldInfo;

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
                appState.realtimeWorld.selectedPlayName = msg.override_name;
            } else {
                appState.realtimeWorld.selectedPlayName = appState.overridePlayInProgress;
            }

            if (msg.names.length > 0) {
                (appState.realtimeWorld.plays as Map<string, Play>).clear();
                for (let i = 0; i < msg.names.length; i++) {
                    if (msg.names[i] in appState.world.plays) {
                        let play = (appState.realtimeWorld.plays as Map<string, Play>).get(msg.names[i]);
                        play.enabled = msg.enableds[i]
                        play.score = msg.scores[i]
                    } else {
                        let play = new Play(
                            msg.names[i],
                            msg.enableds[i],
                            msg.scores[i]
                        );

                        (appState.realtimeWorld.plays as Map<string, Play>).set(play.name, play);
                    }
                }
            }
        }
    }

    worldUpdateFunction(msg, appState: AppState) {
        let world = appState.realtimeWorld;

        if ('timestamp' in msg) {
            appState.realtimeWorld.timestamp = msg.timestamp;
        } else {
            // Convert timestamp to millis
            appState.realtimeWorld.timestamp = (msg.current_time.sec * 1e3) + (msg.current_time.nanosec / 1e6);
        }
        appState.realtimeWorld.fps = msg.fps;

        if (msg.balls.length > 0) {
            appState.realtimeWorld.ball.pose = msg.balls[0].pose;
            appState.realtimeWorld.ball.visible = msg.balls[0].visible;
        }

        for (const team of [TeamColor.Blue, TeamColor.Yellow]) {
            const msgRobotArray = (team === appState.realtimeWorld.team) ? msg.our_robots : msg.their_robots;

            for (let id = 0; id < msgRobotArray.length; id++) {
                let robot = appState.realtimeWorld.teams.get(team).robots[id] as Robot;
                robot.pose = msgRobotArray[id].pose;
                robot.twist = msgRobotArray[id].twist;
                robot.accel = msgRobotArray[id].accel;
                robot.visible = msgRobotArray[id].visible;
            }
        }

        if (appState.useKenobi) {
            // appState.updateHistory();
            appState.updateBagHistory();
        } else {
            // This should never happen
        }
    }

    fieldUpdateFunction(msg, appState: AppState) {
        let world = appState.realtimeWorld;

        let newFieldInfo = new FieldInfo();
        for (const member of Object.getOwnPropertyNames(msg)) {
            newFieldInfo[member] = msg[member];
        }
        world.field.fieldInfo = newFieldInfo;
    }

    playbookUpdateFunction(msg, appState: AppState) {
        let world = appState.realtimeWorld;

        if (appState.overridePlayInProgress == null) {
            world.selectedPlayName = msg.override_name;
        } else {
            world.selectedPlayName = appState.overridePlayInProgress;
        }

        if (msg.names.length > 0) {
            let plays = world.plays as Map<string, Play>;
            plays.clear();
            for (let i = 0; i < msg.names.length; i++) {
                if (msg.names[i] in world.plays) {
                    plays.get(msg.names[i]).enabled = msg.enableds[i]
                    plays.get(msg.names[i]).score = msg.scores[i]
                } else {
                    let play = new Play(
                        msg.names[i],
                        msg.enableds[i],
                        msg.scores[i]
                    )

                    plays.set(play.name, play);
                }
            }
        }
    }

    playInfoUpdateFunction(msg, appState: AppState) {
        let world = appState.realtimeWorld;

        world.ai.name = msg.name;
        world.ai.description = msg.description;
    }

    overlayUpdateFunction(msg, appState: AppState) {
        let overlayMap = appState.realtimeWorld.field.overlays as Map<string, Overlay>;
        const underlayContainer = appState.graphicState.underlayContainer;
        const overlayContainer = appState.graphicState.overlayContainer;
        for (const overlay of msg.overlays) {
            let id = overlay.ns+"/"+overlay.name;

            // Flag the overlay to check if it moved the graphic object between containers
            const checkOtherDepth = (id in appState.realtimeWorld.field.overlays &&
                overlayMap.get(id).depth != overlay.depth);

            switch(overlay.command) {
                // REPLACE
                case 0:
                    overlayMap.set(id, new Overlay(id, overlay, checkOtherDepth, appState.realtimeWorld.timestamp));
                    if (overlay.type == OverlayType.Heatmap) {
                        initializeHeatmapGraphic(overlayMap.get(id), appState.graphicState)
                    }
                    break;
                // EDIT
                case 1:
                    //TODO: Not sure if this command is necessary, will implement later if it is
                    break;
                // REMOVE
                case 2:
                    deleteOverlayGraphic(overlayMap.get(id), underlayContainer);
                    deleteOverlayGraphic(overlayMap.get(id), overlayContainer);
                    overlayMap.delete(id);
                    break;
            }
        }

        if (appState.historyManager.viewingBag) {
            for (const [key, overlay] of overlayMap) {
                if (isOverlayExpired(overlay, appState.realtimeWorld.timestamp)) {
                    let id = overlay.ns+"/"+overlay.name;
                    deleteOverlayGraphic(overlayMap.get(id), underlayContainer);
                    deleteOverlayGraphic(overlayMap.get(id), overlayContainer);
                    overlayMap.delete(id);
                }
            }

        }
    }

    refereeUpdateFunction(msg, appState: AppState) {
        let world = appState.realtimeWorld;

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


    getJoystickStatusCallback(appState: AppState): (msg: any) => void {
        return function(msg: any): void {
            if (msg.is_active) {
                appState.controlledRobot = msg.active_id;
            } else {
                appState.controlledRobot = -1;
            }
        }
    }

    processBagSchema(record, schemasById, appState: AppState) {
        if (record.type != "Schema") {
            return;
        }

        const existingSchema = schemasById.get(record.id);
        if (existingSchema) {
            if ((JSON.stringify(existingSchema) != JSON.stringify(record))) {
                throw new Error(`differing schemas for id ${record.id}`);
            }

            return;
        }
        schemasById.set(record.id, record);
    }

    processBagChannel(record, schemasById, channelInfoById, appState: AppState) {
        if (record.type != "Channel") {
            return;
        }

        const existingInfo = channelInfoById.get(record.id);
        if (existingInfo) {
            if ((JSON.stringify(existingInfo.info) != JSON.stringify(record))) {
                throw new Error(`differing channels for id ${record.id}`);
            }
            return;
        }
        
        if (record.schemaId === 0) {
            throw new Error(
                `Channel ${record.id} has no schema; channels without schemas are not supported`,
            );
        }

        const schema = schemasById.get(record.schemaId);
        if (!schema) {
            throw new Error(`Missing schema ${record.schemaId} for channel ${record.id}`);
        }

        if (record.messageEncoding === "cdr") {
            let reader: MessageReader;
            if (schema.encoding === "ros2msg") {
                reader = new MessageReader(
                    parseMsgDefinition(new TextDecoder().decode(schema.data), {
                    ros2: true,
                    }),
                );
            } else if (schema.encoding === "ros2idl") {
                reader = new MessageReader(
                    parseIdlDefinition(new TextDecoder().decode(schema.data)),
                );
            }

            let messageDeserializer: (data: ArrayBufferView) => unknown;
            messageDeserializer = (data) => reader.readMessage(data);
            channelInfoById.set(record.id, { info: record, messageDeserializer });
        }
    }

    processBagMessage(record, channelInfoById, appState: AppState) {
        if (record.type != "Message") {
            return;
        }

        const channelInfo = channelInfoById.get(record.channelId);
        if (!channelInfo) {
            console.log("no channel info for: ", record);
            // throw new Error(`message for channel ${record.channelId} with no prior channel info`);
            return;
        }
        if (channelInfo.messageDeserializer == undefined) {
            console.log("no message deserializer for: ", record);
            return;
            // throw new Error(
            //     `No deserializer available for channel id: ${channelInfo.info.id} ${channelInfo.info.messageEncoding}`,
            // );
        }

        const message = channelInfo.messageDeserializer(record.data);
        switch(channelInfo.info.topic) {
            case "/kenobi_node/world":
                message.timestamp = Number(record.logTime / BigInt(1e9));
                this.worldUpdateFunction(message, appState);
                break;
            case "/kenobi_node/playbook_state":
                this.playbookUpdateFunction(message, appState);
                break;
            case "/referee_messages":
                this.refereeUpdateFunction(message, appState);
                break;
            case "/play_info":
                this.playInfoUpdateFunction(message, appState);
                break;
            case "/overlays":
                this.overlayUpdateFunction(message, appState);
                break;
            case "/field":
                this.fieldUpdateFunction(message, appState);
                break;
        }
    }

    async loadBagFile(appState: AppState, file: Blob) {

        appState.historyManager.viewingBag = true;
        appState.historyManager.historyEndIndex = -1;
        appState.historyManager.selectedHistoryFrame = -1;
        // appState.historyManager.historyReplayIsPaused = true;


        // let hist = appState.historyManager.compressedWorldHistory
        // hist = new Array(100000);

        // hist = new Array(100000).fill(null).map(() => (new WorldState()));
        // hist = new Array(100000);
        // for (let i = 0; i < hist.length; i++) {
        //     // if (i < 25000 || i > 75000) {
        //     if (true) {
        //         hist[i] = new WorldState();
        //         hist[i].timestamp = i;
        //     }
        // }
        // hist.slice(25000, 75000).forEach((_, index) => ( delete hist[25000 + index]))


        // Filled Array
        // Performance Results (clearing entire array):
        // hist.slice(0, 100000).forEach((_, index, arr) => ( delete hist[index]))
        //      Time: 0.0025, 0.0026 | 0.00219
        // for of loop
        //      Time: 0.003699

        // Sparse Array (half filled)
        // Performance Results (clearing entire array):
        // hist.slice(0, 100000).forEach((_, index, arr) => ( delete hist[index]))
        //      Time: 0.00279 | 0.00169
        // filter to for of loop
        //      Time: 0.00769

        // let sl = hist.filter((elem) => (elem.timestamp < 25000));
        // console.log("SL: ", sl.length)
        // let num = 0;
        // const new_proc_start_time = performance.now();

        // Slice/For Each
        // hist.slice(0, 100000).forEach((elem, index) => { if (elem.timestamp < 2500) {delete hist[index]} num++});

        // For of Loop
        // for (let [index, elem] of sl.entries()) {
        //     // if (elem.timestamp > 2500) {
        //     //     break;
        //     // }
        //     delete hist[elem.timestamp];
        //     num++;
        // }

        // console.log("Processing took: ", (performance.now() - new_proc_start_time) / 1000.0, "s");
        // console.log("Processed ", num, " values")
        // console.log(hist)
        // return;


        const reader = await McapIndexedReader.Initialize({
            readable: new BlobReadable(file)
        });

        console.log("reader loaded");
        console.log(reader);

        const schemasById = new Map<number, McapTypes.TypedMcapRecords["Schema"]>();
        const channelInfoById = new Map<
            number,
            {
            info: McapTypes.Channel;
            messageDeserializer?: (data: ArrayBufferView) => unknown;
            }
        >();

        console.log("schemabyid:");
        console.log(reader.schemasById);

        console.log("reading channels")
        for (const record of reader.channelsById.values()) {
            this.processBagChannel(record, reader.schemasById, channelInfoById, appState);
        }
        console.log("channelbyid:");
        console.log(channelInfoById);

        const message_start_time = reader.statistics.messageStartTime / BigInt(1e9);
        const message_end_time = reader.statistics.messageEndTime / BigInt(1e9);
        const bag_length_time = Number(message_end_time - message_start_time);

        const num_frames = Math.ceil(bag_length_time * 100.0);
        console.log("Setting History Array Length to: ", num_frames);
        // appState.historyManager.compressedWorldHistory = new Array(num_frames);
        appState.historyManager.compressedWorldHistory = [];
        appState.historyManager.loadedFrames = 0;
        appState.historyManager.historyEndIndex = -1;
        appState.historyManager.bagStartTime = Number(message_start_time);
        appState.historyManager.bagEndTime = Number(message_end_time);
        console.log("setting history length: ", appState.historyManager.compressedWorldHistory);

        const processing_start_time = Date.now();
        let last_print_time = Date.now();
        let index = 0;
        // console.log("Loading Ref Data");
        // for await (const record of reader.readMessages({
        //     topics: [
        //         "/referee_messages",
        //         // "/kenobi_node/world",
        //     ]
        // })) {

        //     const channelInfo = channelInfoById.get(record.channelId);
        //     const message: any = channelInfo.messageDeserializer(record.data);

        //     // KENOBI WORLD MESSAGE
        //     // appState.refHistory.push(
        //     //     new RefereeHistory(
        //     //         message.referee_info.game_stage,
        //     //         message.referee_info.game_command,
        //     //         message.ball_in_play,
        //     //         Number((record.logTime / BigInt(1e9)) - message_start_time)
        //     //     )
        //     // );

        //     // REFEREE MESSAGE
        //     appState.historyManager.refHistory.push(
        //         new RefereeHistory(
        //             message.stage,
        //             message.command,
        //             false,
        //             Number((record.logTime / BigInt(1e9)) - message_start_time)
        //         )
        //     );

        //     if (Date.now() - last_print > 1000) {
        //         console.log((record.logTime / BigInt(1e9)) - message_start_time);
        //         last_print = Date.now();
        //         console.log("ref_test length: ", appState.historyManager.refHistory.length)

        //         // if ((record.logTime / BigInt(1e9)) - message_start_time > 100) {
        //         //     break;
        //         // }
        //     }
        // }

        // console.log("load ref time: ", (Date.now() - start) / 1000.0);

        // const canvas = document.createElement('canvas');
        // canvas.width = 1000;
        // canvas.height = 200;
        // const ctx = canvas.getContext('2d');

        // const timestep = appState.historyManager.refHistory[appState.historyManager.refHistory.length - 1].history_timestamp / canvas.width;
        // let prev_ref_index = 0;
        // for (let i = 0; i < canvas.width; i++) {
        //     const index_time = appState.historyManager.refHistory[0].history_timestamp + (i * timestep);
        //     let ref_frame = appState.historyManager.refHistory[prev_ref_index];

        //     for (let ref_index = prev_ref_index; ref_index < appState.historyManager.refHistory.length; ref_index++) {
        //         if (appState.historyManager.refHistory[ref_index].history_timestamp >= index_time) {
        //             ref_frame = appState.historyManager.refHistory[ref_index];
        //             prev_ref_index = ref_index;
        //             break;
        //         }
        //     }

        //     if (ref_frame.ball_in_play) {
        //         ctx.fillStyle = 'green';
        //     } else {
        //         ctx.fillStyle = GameCommandProperties[ref_frame.command].color;
        //     }
        //     ctx.fillRect(
        //         i,
        //         0,
        //         1,
        //         canvas.height
        //     );
        // }

        // document.body.prepend(canvas);

        // console.log("total time to load and display ref data: ", (Date.now() - start) / 1000.0)

        for await (const record of reader.readMessages({
            startTime: reader.statistics.messageStartTime,
            topics: [
                "/kenobi_node/world",
                "/kenobi_node/playbook_state",
                "/referee_messages",
                "/play_info",
                "/overlays",
                "/field",
            ]
        })) {
            this.processBagMessage(record, channelInfoById, appState);

            if (Date.now() - last_print_time > 1000) {
                console.log(Number((record.logTime / BigInt(1e9)) - message_start_time), "/", bag_length_time, " | ", appState.historyManager.loadedFrames);
                const first = appState.historyManager.compressedWorldHistory.find((elem) => (elem != undefined));
                console.log("First loaded time: ", first.timestamp - Number(message_start_time));
                last_print_time = Date.now();

                // if ((record.logTime / BigInt(1e9)) - message_start_time > 50) {
                //     break;
                // }
            }
        }

        console.log("completion time: ", (Date.now() - processing_start_time) / 1000.0);
        console.log(appState);
        console.log(appState.historyManager.compressedWorldHistory);
    }
}
