import { Overlay, updateOverlay } from "@/overlay";
import { AppState } from "@/state";
import * as PIXI from "pixi.js";
import { Viewport } from "pixi-viewport";
import { drawBall, updateBall } from "@/ball";
import { drawRobot, updateRobot } from "@/robot";

// Types used for field data and overlays

export class FieldDimensions {
    length: number = 12;
    width: number = 9;
    border: number = 0.7;
    lineWidth: number = 0.01;
    goalWidth: number = 1.8;
    goalDepth: number = 0.18;
    goalHeight: number = 0.16;
    penaltyShort: number = 1.8;
    penaltyLong: number = 3.6;
    centerRadius: number = 0.5;
    centerDiameter: number = 1;
    goalFlat: number = 0.5;
    floorLength: number = 13.4;
    floorWidth: number = 10.4
}

export class FieldSidedInfo {
    defense_area_corners: Point[];
    goal_corners: Point[];
}

export class Field {
    fieldDimensions: FieldDimensions;
    overlays: Map<string, Overlay> = new Map<string, Overlay>;

    constructor() {
        this.fieldDimensions = new FieldDimensions();
    }
}

export function drawSideIgnoreOverlay(state: AppState, fieldUI: PIXI.Container) {
    const field = state.world.field;
    const scale = state.renderConfig.scale;

    let ignoreOverlay = fieldUI.getChildByName("ignoreOverlay") as PIXI.Graphics;
    if (!ignoreOverlay) {
        ignoreOverlay = new PIXI.Graphics;
        ignoreOverlay.name = "ignoreOverlay";
        ignoreOverlay.visible = false;
        fieldUI.addChild(ignoreOverlay);
    }

    ignoreOverlay.clear();
    ignoreOverlay.beginFill("#3b3b3bFF");
    ignoreOverlay.drawRect(0,
                    -scale * (field.fieldDimensions.width/2 + field.fieldDimensions.border),
                    scale * (field.fieldDimensions.length/2 + field.fieldDimensions.border),
                    scale * (field.fieldDimensions.width + 2*field.fieldDimensions.border),
                    );


    let hoverOverlay = fieldUI.getChildByName("hoverOverlay") as PIXI.Graphics;
    if (!hoverOverlay) {
        hoverOverlay = new PIXI.Graphics;
        hoverOverlay.name = "hoverOverlay";
        hoverOverlay.visible = false;
        fieldUI.addChild(hoverOverlay);
    }

    hoverOverlay.clear();
    hoverOverlay.beginFill("#3b3b3bBF");
    hoverOverlay.drawRect(0,
                    -scale * (field.fieldDimensions.width/2 + field.fieldDimensions.border),
                    scale * (field.fieldDimensions.length/2 + field.fieldDimensions.border),
                    scale * (field.fieldDimensions.width + 2*field.fieldDimensions.border),
                    );

}

export function drawFieldBoundary(state: AppState, fieldBoundary: PIXI.Graphics) {
    const field = state.world.field;
    const scale = state.renderConfig.scale;

    fieldBoundary.clear();

    // Draw the Field Boundaries
    const boundaryStrokeWidth = 8;
    fieldBoundary.lineStyle(boundaryStrokeWidth, 0x000000);
    // Field Outline
    fieldBoundary.drawRect(-(boundaryStrokeWidth / 2) - scale * (field.fieldDimensions.border + (field.fieldDimensions.length / 2)),
        -(boundaryStrokeWidth / 2) - scale * (field.fieldDimensions.border + (field.fieldDimensions.width / 2)),
        boundaryStrokeWidth + ((field.fieldDimensions.length + 2 * field.fieldDimensions.border) * scale),
        boundaryStrokeWidth + ((field.fieldDimensions.width + 2 * field.fieldDimensions.border) * scale)
    );
}

export function drawFieldLines(state: AppState, fieldLines: PIXI.Graphics) {
    const field = state.world.field;
    const scale = state.renderConfig.scale;

    fieldLines.clear();

    // Draw the Field Lines
    fieldLines.lineStyle(4, 0xFFFFFF);

    // Field Outline
    fieldLines.drawRect(-scale * field.fieldDimensions.length / 2,
        -scale * field.fieldDimensions.width / 2,
        field.fieldDimensions.length * scale,
        field.fieldDimensions.width * scale
    );

    // Center Circle
    fieldLines.drawCircle(0, 0, scale * field.fieldDimensions.centerRadius);

    // Width Center Line
    fieldLines.moveTo(0, -scale * field.fieldDimensions.width / 2);
    fieldLines.lineTo(0, scale * field.fieldDimensions.width / 2);

    // Length Center Line
    fieldLines.moveTo(-scale * field.fieldDimensions.length / 2, 0);
    fieldLines.lineTo(scale * field.fieldDimensions.length / 2, 0);

    // Team Goal Boxes
    for (const color of state.world.teams.keys()) {
        const team = state.world.teams.get(color);
        const direction = (color == state.world.team ? -1 : 1); // always draw our goal on the left
        const goalX = direction * scale * field.fieldDimensions.length / 2

        // Goal Box
        fieldLines.lineStyle(4, 0xFFFFFF);
        fieldLines.moveTo(goalX, -scale * field.fieldDimensions.penaltyLong / 2);
        fieldLines.lineTo(goalX - direction * scale * field.fieldDimensions.penaltyShort, -scale * field.fieldDimensions.penaltyLong / 2);
        fieldLines.lineTo(goalX - direction * scale * field.fieldDimensions.penaltyShort, scale * field.fieldDimensions.penaltyLong / 2);
        fieldLines.lineTo(goalX, scale * field.fieldDimensions.penaltyLong / 2);

        // Goal
        fieldLines.lineStyle(4, color);
        fieldLines.moveTo(goalX, -scale * field.fieldDimensions.goalWidth / 2);
        fieldLines.lineTo(goalX + direction * scale * field.fieldDimensions.goalDepth, -scale * field.fieldDimensions.goalWidth / 2);
        fieldLines.lineTo(goalX + direction * scale * field.fieldDimensions.goalDepth, scale * field.fieldDimensions.goalWidth / 2);
        fieldLines.lineTo(goalX, scale * field.fieldDimensions.goalWidth / 2);
    }
}

export function drawRobots(state: AppState, robotsContainer: PIXI.Container) {
    const robotArray = Array.from(state.world.teams.values()).map(i => { return i.robots }).flat()
    for (var i = 0; i < robotArray.length; i++) {
        drawRobot(robotArray[i], robotsContainer, state.renderConfig);
    }
}

export function initializePixi(app: PIXI.Application, state: AppState): PIXI.Container {
    const field = state.world.field;

    // create viewport
    const viewport = new Viewport({
        screenWidth: 1876,
        screenHeight: 1456,
        worldWidth: 1876,
        worldHeight: 1456,
        passiveWheel: false,
        stopPropagation: true,
        disableOnContextMenu: true,

        events: app.renderer.events // the interaction module is important for wheel to work properly when renderer.view is placed or scaled
    })

    // add the viewport to the stage
    viewport.name = "viewport";
    const offsetX = app.screen.width / 2;
    const offsetY = app.screen.height / 2;
    viewport.position.set(offsetX, offsetY);
    app.stage.addChild(viewport);

    // activate plugins
    viewport
        .drag({
            mouseButtons: "left"
        })
        .pinch()
        .wheel()

    const fieldContainer = new PIXI.Container();

    const ballVelLine = new PIXI.Graphics();
    ballVelLine.name = "ballVelLine";
    app.stage.addChild(ballVelLine);


    const fieldLines = new PIXI.Graphics();
    fieldLines.name = "fieldLines";

    const fieldBoundary = new PIXI.Graphics();
    fieldBoundary.name = "fieldBoundary";

    const robots = new PIXI.Container();
    robots.name = "robots";

    const ball = new PIXI.Container();
    ball.name = "ball";

    const underlay = new PIXI.Container();
    underlay.name = "underlay";
    state.graphicState.underlayContainer = underlay;
    const overlay = new PIXI.Container();
    overlay.name = "overlay";
    state.graphicState.overlayContainer = overlay;

    const fieldUI = new PIXI.Container();
    fieldUI.name = "fieldUI";

    // Field Lines
    drawFieldLines(state, fieldLines);

    // Field Boundary Walls
    drawFieldBoundary(state, fieldBoundary);

    // Rectangle that covers the ignored side of the field
    drawSideIgnoreOverlay(state, fieldUI);

    // Robots
    drawRobots(state, robots);

    // Ball
    drawBall(state.world.ball, ball, state.renderConfig);

    fieldContainer.addChild(fieldLines);
    fieldContainer.addChild(fieldBoundary);
    fieldContainer.addChild(underlay);
    fieldContainer.addChild(robots);
    fieldContainer.addChild(ball);
    fieldContainer.addChild(overlay);
    fieldContainer.addChild(fieldUI);

    viewport.addChild(fieldContainer);

    return fieldContainer;
}

export function updateField(state: AppState, fieldContainer: PIXI.Container) {
    const field = state.world.field;

    const robotArray = Array.from(state.world.teams.values()).map(i => { return i.robots }).flat()
    const robots = fieldContainer.getChildByName("robots").children;
    for (var i = 0; i < robotArray.length; i++) {
        if (i != state.draggedRobot) {
            const robotContainer = robots[i] as PIXI.Container;
            updateRobot(robotArray[i], robotContainer, state.renderConfig);
        }
    }

    updateBall(state.world.ball, fieldContainer.getChildByName("ball").children[0] as PIXI.Container, state.renderConfig);

    for (const [id, overlay] of field.overlays) {
        const shouldDelete = updateOverlay(overlay, state.world.timestamp, fieldContainer.getChildByName("overlay"), fieldContainer.getChildByName("underlay"), state.renderConfig);

        if (shouldDelete) {
            field.overlays.delete(id);
        }
    }
}