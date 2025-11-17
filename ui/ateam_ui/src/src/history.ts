import ROSLIB from "roslib";
import { toRaw } from "vue";
import { WorldState, AppState, StringCacheItem } from "@/state";
import { Robot } from "@/robot";
import { Team, TeamColor } from "@/team";
import { Referee, RefereeHistory } from "@/referee";
import { Ball } from "@/ball";
import { Field } from "@/field";
import { AIState } from "@/AI";
import { Play, compressedArrayToPlayMap, playMapToCompressedArray } from "@/play";
import { Overlay, OverlayType } from "@/overlay";

export class HistoryManager {
    refHistory: RefereeHistory[] = [];
    compressedWorldHistory: WorldState[] = []; // Treated as a circular buffer
    frameLimit: number = 100000;
    loadedFrames: number;

    historyEndIndex: number = -1; // Index of the last element of circular buffer
    selectedHistoryFrame: number = -1; // Goes from 0 to length of the history buffer linearly, not using the circular buffer index system.
    historyReplayIsPaused: boolean = true;

    viewingBag: boolean = false;
    bagStartTime: number = null;
    bagEndTime: number = null;

    updateHistory(appState: AppState): void {
        // Only store history while we are unpaused
        if (this.selectedHistoryFrame == -1 || this.viewingBag) {
            this.historyEndIndex++;
            let compressed_world = this.compressWorldState(appState);
            if (this.compressedWorldHistory.length < this.frameLimit) {
                this.compressedWorldHistory.push(compressed_world);
            } else {
                if (this.historyEndIndex >= this.compressedWorldHistory.length) {
                    this.historyEndIndex = 0;
                }

                this.compressedWorldHistory[this.historyEndIndex] = compressed_world;
            }
        }
    }

    updateBagHistory(appState: AppState): void {
        if (this.viewingBag) {
            return;
        }

        this.historyEndIndex++;
        this.loadedFrames++;

        // // TODO: this may end up behaving weird if we are not hitting the kenobi framerate
        // if (this.loadedFrames > this.frameLimit) {
        //     let start_time = 0.0
        //     let end_time = appState.realtimeWorld.timestamp - this.bagStartTime - 10.0;
        //     if (end_time < 0) {
        //         console.log("ran out of space while near start of buffer ??????");
        //         // TODO: handle this way better
        //         start_time = end_time + 20.0;
        //         end_time = this.bagEndTime;
        //     }

        //     let filtered = this.compressedWorldHistory.filter((elem) => (elem.timestamp < end_time));
        //     for (let [_, elem] of filtered.entries()) {
        //         if (this.loadedFrames < this.frameLimit - 1000) {
        //             break;
        //         }

        //         const index = Math.ceil((elem.timestamp - this.bagStartTime) / 100.0);
        //         delete this.compressedWorldHistory[elem.timestamp];
        //         this.loadedFrames--;
        //     }

        // }

        let compressed_world = this.compressWorldState(appState);
        // this.compressedWorldHistory[this.historyEndIndex] = compressed_world;
        this.compressedWorldHistory.push(compressed_world);
    }

    compressWorldState(appState: AppState) {
        let compressedWorld: WorldState;
        try {
            compressedWorld = structuredClone(appState.realtimeWorld);
        } catch(error) {
            if (error instanceof DOMException && error.name === 'DataCloneError') {
                compressedWorld = structuredClone(toRaw(appState.realtimeWorld));
            }
        }

        compressedWorld.teams.get(TeamColor.Blue).robots =
            robotArrayToCompressed(compressedWorld.teams.get(TeamColor.Blue).robots as Robot[]);
        compressedWorld.teams.get(TeamColor.Yellow).robots =
            robotArrayToCompressed(compressedWorld.teams.get(TeamColor.Yellow).robots as Robot[]);

        compressedWorld.plays =
            playMapToCompressedArray(compressedWorld.plays as Map<string, Play>, appState.playNameCache);
        compressedWorld.field.overlays =
            overlaysToCompressedArray(
                compressedWorld.field.overlays as Map<string, Overlay>,
                appState.overlayNamespaceCache,
                appState.overlayNameCache
            );

        return compressedWorld;
    }

    decompressWorldState(compressedWorld: WorldState, appState: AppState) {
        let world: WorldState;
        try {
            world = structuredClone(compressedWorld);
        } catch(error) {
            if (error instanceof DOMException && error.name === 'DataCloneError') {
                world = structuredClone(toRaw(compressedWorld));
            }
        }

        world.teams.get(TeamColor.Blue).robots = compressedToRobotArray(TeamColor.Blue,
            world.teams.get(TeamColor.Blue).robots as number[]);
        world.teams.get(TeamColor.Yellow).robots = compressedToRobotArray(TeamColor.Yellow,
            world.teams.get(TeamColor.Yellow).robots as number[]);
        world.plays = compressedArrayToPlayMap(world.plays as number[], appState.playNameCache);
        world.field.overlays = compressedArrayToOverlays(world.field.overlays as Overlay[],
            appState.overlayNamespaceCache, appState.overlayNameCache);

        return world;
    }
}

export function robotArrayToCompressed(robotArray: Robot[]): number[] {

    const compressedArray: number[][] = 
        robotArray.map((robot) => {

            return [
                robot.pose.position.x,
                robot.pose.position.y,
                robot.pose.position.z,
                robot.pose.orientation.z, // Only need z for yaw
                robot.twist.linear.x,
                robot.twist.linear.y,
                robot.twist.angular.z,
                Number(robot.radio_connected),
                Number(robot.visible)
            ];
        });

    return compressedArray.flat();
}

export function compressedToRobotArray(color: TeamColor, compressedArray: number[]): Robot[] {
    const robots: Robot[] = [];
    for (let i = 0; i < compressedArray.length; i += 9) {

        let pose = new ROSLIB.Pose({
            position: {
                x: compressedArray[i],
                y: compressedArray[i+1],
                z: compressedArray[i+2]
            },
            orientation: {
                x: 0,
                y: 0,
                z: compressedArray[i+3],
                w: 0
            }
        })

        let robot = new Robot(
            i/9,
            Boolean(compressedArray[i+8]),
            color,
            pose
        );

        robot.twist = {
            linear: new ROSLIB.Vector3({
                x: compressedArray[i+4],
                y: compressedArray[i+5],
                z: 0
            }),
            angular: new ROSLIB.Vector3({
                x: 0,
                y: 0,
                z: compressedArray[i+6]
            })
        }

        robot.radio_connected = Boolean(compressedArray[i+7]);

        robots.push(robot);
    }

    return robots;
}

export function overlaysToCompressedArray(overlayMap: Map<string, Overlay>, 
    overlayNamespaceCache: StringCacheItem[], overlayNameCache: StringCacheItem[]): Overlay[] {

    const namespaceList = overlayNamespaceCache.map(function(obj: StringCacheItem) {
        return obj.name;
    });
    const nameList = overlayNameCache.map(function(obj: StringCacheItem) {
        return obj.name;
    });

    let compressedArray: Overlay[] = [];

    for (const [name, overlay] of overlayMap) {
        if (overlay.type == OverlayType.Heatmap) {
            continue;
        }

        let overlayNamespaceIndex = namespaceList.indexOf(overlay.ns as string);
        if (overlayNamespaceIndex == -1) {
            namespaceList.push(overlay.ns as string);
            overlayNamespaceCache.push(new StringCacheItem(overlay.ns as string));
            overlayNamespaceIndex = overlayNamespaceCache.length - 1;
        }
        
        let overlayNameIndex = nameList.indexOf(overlay.name as string);
        if (overlayNameIndex == -1) {
            nameList.push(overlay.name as string);
            overlayNameCache.push(new StringCacheItem(overlay.name as string));
            overlayNameIndex = overlayNameCache.length - 1;
        }

        let compressedOverlay = compressOverlay(overlayNamespaceIndex, overlayNameIndex, overlay);
        compressedArray.push(compressedOverlay);
    }

    return compressedArray;
}

export function compressedArrayToOverlays(compressedArray: Overlay[],
    overlayNamespaceCache: StringCacheItem[], overlayNameCache: StringCacheItem[]): Map<string, Overlay> {

    let overlayMap: Map<string, Overlay> = new Map();

    for (const compressedOverlay of compressedArray) {
        let namespace = overlayNamespaceCache[compressedOverlay.name];
        let name = overlayNameCache[compressedOverlay.name];

        let overlay = decompressOverlay(namespace, name, compressedOverlay);
        overlayMap.set(overlay.id,
            overlay
        );
    }

    return overlayMap;
}

function compressOverlay(overlayNamespaceIndex: number, overlayNameIndex: number, overlay: Overlay): Overlay {
    let compressedOverlay = structuredClone(overlay);

    delete compressedOverlay.id;
    compressedOverlay.ns = overlayNamespaceIndex;
    compressedOverlay.name = overlayNameIndex;

    delete compressedOverlay.heatmap_data;
    delete compressedOverlay.heatmap_alpha;
    delete compressedOverlay.heatmap_resolution_width;
    delete compressedOverlay.heatmap_resolution_height;

    return compressedOverlay;
}

function decompressOverlay(namespace: string, name: string, compressedOverlay: Overlay): Overlay {
    let overlay = structuredClone(compressedOverlay);
    overlay.id = namespace + "/" + name;
    overlay.ns = namespace;
    overlay.name = name;

    return overlay;
}