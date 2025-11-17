import { Plugin } from './Plugin';
import type { Container, PointData } from 'pixi.js';
import type { Viewport } from '../Viewport';
export interface IFollowOptions {
    speed?: number;
    acceleration?: number | null;
    radius?: number | null;
}
export declare class Follow extends Plugin {
    readonly options: Required<IFollowOptions>;
    target: Container;
    protected velocity: PointData;
    constructor(parent: Viewport, target: Container, options?: IFollowOptions);
    update(elapsed: number): void;
}
