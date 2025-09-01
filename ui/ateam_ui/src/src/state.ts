import ROSLIB from "roslib";
import { Team, TeamColor } from "@/team";
import { Referee } from "@/referee";
import { Ball } from "@/ball";
import { Field } from "@/field";
import { AIState } from "@/AI";
import { Play } from "@/play";
import { BackendManager, MessageType } from "./backendManager";
import * as PIXI from "pixi.js";

export class RenderConfig {
    angle: number = 0; // Rotation applied to the rendered field
    scale: number = 140; // Pixels per meter (in the rendering canvas)   
}

export class WorldState {
    teamName: string;
    team: TeamColor;
    teams: Map<TeamColor, Team> = new Map<TeamColor, Team>;
    ball: Ball;
    field: Field;
    referee: Referee;
    ai: AIState;
    plays: Map<string, Play> = new Map<string, Play>;
    selectedPlayName: string = null;
    overridePlay: string;
    ball_in_play: boolean;
    double_touch_enforced: boolean;
    double_touch_id: number;

    timestamp: number;
    fps: number;

    constructor() {
        this.teamName = "A-Team";
        this.team = TeamColor.Blue;
        this.teams.set(TeamColor.Blue, new Team(this.teamName, TeamColor.Blue, -1));
        this.teams.set(TeamColor.Yellow, new Team("Uknown", TeamColor.Yellow, 1));

        this.ball = new Ball();
        this.field = new Field();

        this.referee = new Referee();
        this.ai = new AIState();
        this.overridePlay = null;
        this.ball_in_play = false;
        this.double_touch_enforced = false;
        this.double_touch_id = -1;

        this.timestamp = Date.now();
        this.fps = 100;
    }
}

export class GraphicState {
    fieldUpdateFunction: () => void
    fieldContainer: PIXI.Container
    overlayContainer: PIXI.Container
    underlayContainer: PIXI.Container
    sslVisionContainers: Map<string, PIXI.Graphics> = new Map<string, PIXI.Graphics>;
}

export class AppState {
    backendManager: BackendManager;

    renderConfig: RenderConfig;
    graphicState: GraphicState;

    lastTimeReceivedKenobi: number;
    realtimeWorld: WorldState; // World that is constantly kept up to date by ROS callbacks
    world: WorldState; // World to be displayed, can be set to a world state in the past history buffer

    worldHistory: WorldState[] = []; // Treated as a circular buffer
    historyEndIndex: number = -1; // Index of the last element of circular buffer
    selectedHistoryFrame: number = -1; // Goes from 0 to length of the history buffer linearly, not using the circular buffer index system.
    historyReplayIsPaused: boolean = true;

    // TODO: Add ROS params for these
    sim: boolean = true;
    comp: boolean = false;

    useKenobi: boolean = true; // Specifies whether to use the Kenobi world topic or ros robot/ball state topics for world data

    controlledRobot: number = null;

    overridePlayInProgress: string = null;

    hoveredFieldIgnoreSide: number = 0;
    draggedRobot: number = null;

    goalieServiceStatus: [status: boolean, reason: string] = [true, ""] // False if the set goalie service fails

    constructor() {
        this.renderConfig = new RenderConfig();
        this.graphicState = new GraphicState();

        this.realtimeWorld = new WorldState();
        this.world = this.realtimeWorld;

        this.lastTimeReceivedKenobi = Date.now();
    }

    connectToRos(): void {
        this.backendManager = new BackendManager(this);
    }

    updateHistory(): void {
        // Only store history while we are unpausued
        if (this.selectedHistoryFrame == -1) {
            this.historyEndIndex++;
            if (this.worldHistory.length < 100000) {
                // @ts-ignore
                this.worldHistory.push(structuredClone(this.realtimeWorld.__v_raw));
            } else {
                if (this.historyEndIndex >= this.worldHistory.length) {
                    this.historyEndIndex = 0;
                }
                // @ts-ignore
                this.worldHistory[this.historyEndIndex] = structuredClone(this.realtimeWorld.__v_raw);
            }
        }
    }

    setGoalie(goalieId: number): void {
        const state = this;

        const request = {
            msg_type: MessageType.SetDesiredKeeper,
            desired_keeper: goalieId
        };

        this.backendManager.sendBackendCommand(request);
        // this.backendManager.services.get("setGoalie").callService(request,
        //     // Service Response Callback
        //     function(result: any): void {
        //         if(!result.success) {
        //             state.goalieServiceStatus = [false, result.reason];
        //         } else {
        //             state.goalieServiceStatus = [true, ""];
        //         }
        //     },
        //     // Failed to call service callback
        //     function(result: any): void {
        //         state.goalieServiceStatus = [false, result];
        //     }
        // );
    }

    getGoalie(): string {
        // Using current world since the goalie UI element should be based on up to date data from the game controller
        if (this.realtimeWorld.referee && this.realtimeWorld.referee[this.realtimeWorld.team]) {
            const id = this.realtimeWorld.referee[this.realtimeWorld.team].goalkeeper;
            if (id == null || isNaN(id)) {
                return "X";
            }

            return String(id);
        }

        return "X";
    }

    setJoystickRobot(id: number): void {
        // Toggle the selected robot off if it is the same
        if (this.controlledRobot == id) {
            this.controlledRobot = -1;
        } else {
            this.controlledRobot = id;
        }

        this.backendManager.sendBackendCommand({
            msg_type: MessageType.SetJoystickRobot,
            robot_id: this.controlledRobot
        });
        // this.backendManager.params.get("joystick_param").set(this.controlledRobot,
        //     function(result: any): void {}
        // );
    }

    setPlayEnabled(play: Play): void {
        const state = this;
        const request = {
            msg_type: MessageType.SetPlayEnabled,
            play_name: play.name,
            enabled: play.enabled
        };

        this.backendManager.sendBackendCommand(request);
        // this.backendManager.services.get("setPlayEnabled").callService(request,
        //     function(result: any): void {
        //         if(!result.success) {
        //             console.log("Failed to enable/disable ", play.name, ": ", result.reason);
        //         }
        //     });
    }

    setOverridePlay(playName: string): void {
        const state = this;
        const request = {
            msg_type: MessageType.SetOverridePlay,
            play_name: playName
        };

        state.overridePlayInProgress = playName;
        this.backendManager.sendBackendCommand(request);
        // this.backendManager.services.get("setOverridePlay").callService(request,
        //     function(result: any): void {
        //         state.overridePlayInProgress = null;
        //         if(!result.success) {
        //             console.log("Failed to set override play to ", playName, ": ", result.reason);
        //         }
        //     });
    }

    setIgnoreFieldSide(ignoreSide: number): void {
        let intSide = 0;
        if (ignoreSide > 0) {
            intSide = 1;
        } else if (ignoreSide < 0) {
            intSide = -1;
        }

        const request = {
            msg_type: MessageType.SetIgnoreFieldSide,
            ignore_side: intSide
        };

        this.backendManager.sendBackendCommand(request);
        // this.backendManager.services.get("setIgnoreFieldSide").callService(request,
        //     function(result: any): void {
        //         if(!result.success) {
        //             console.error("Failed to set ignore side: ", result.reason);
        //         }
        //     });
    }

    sendSimulatorControlPacket(simulatorControlPacket: any): void {
        const request = {
            msg_type: MessageType.SendSimulatorControlPacket,
            simulator_control: simulatorControlPacket
        };

        this.backendManager.sendBackendCommand(request);
        // this.backendManager.services.get("sendSimulatorControlPacket").callService(request,
        //     function(result: any): void {
        //         if(!result.success) {
        //             console.log("Failed to send simulator packet: ", result.reason);
        //         }
        //     });
    }

    sendPowerRequest(robot_id: number, request_type: string): void {
        let request_int = 0;
        if (request_type == 'shutdown') {
            request_int = 1;
        }

        const request = {
            msg_type: MessageType.SendPowerRequest,
            robot_id: robot_id,
            request_type: request_int
        };

        this.backendManager.sendBackendCommand(request);
        // this.backendManager.services.get("sendPowerRequest").callService(request,
        //     function(result: any): void {
        //         if(!result.success) {
        //             console.log("Failed to send power request packet: ", result.reason);
        //         }
        //     });
    }

    sendRebootKenobiRequest(): void {

        const request = {
            msg_type: MessageType.SendRebootKenobiRequest
        };

        this.backendManager.sendBackendCommand(request);
        // this.backendManager.services.get("sendRebootKenobiRequest").callService(request,
        //     function(result: any): void {
        //         if(!result.success) {
        //             console.log("General Kenobi has escaped: ", result.reason);
        //         }
        //     });
    }

    setUseKenobiTopic(enable: boolean) {
        if (enable != this.useKenobi) {
            if (enable) {
                // this.backendManager.disableStateTopics(this);
                // this.backendManager.enableKenobiTopic(this);
            } else {
                // this.backendManager.disableKenobiTopic(this);
                // this.backendManager.enableStateTopics(this);
            }
        }

        this.useKenobi = enable;
    }
}