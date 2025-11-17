import { SHAPES } from '@pixi/core';
import { buildCircle } from './buildCircle.mjs';
export { buildCircle } from './buildCircle.mjs';
import { buildPoly } from './buildPoly.mjs';
export { buildPoly } from './buildPoly.mjs';
import { buildRectangle } from './buildRectangle.mjs';
export { buildRectangle } from './buildRectangle.mjs';
import { buildRoundedRectangle } from './buildRoundedRectangle.mjs';
export { buildRoundedRectangle } from './buildRoundedRectangle.mjs';
export { ArcUtils } from './ArcUtils.mjs';
export { BatchPart } from './BatchPart.mjs';
export { BezierUtils } from './BezierUtils.mjs';
export { buildLine } from './buildLine.mjs';
export { QuadraticUtils } from './QuadraticUtils.mjs';

const FILL_COMMANDS = {
  [SHAPES.POLY]: buildPoly,
  [SHAPES.CIRC]: buildCircle,
  [SHAPES.ELIP]: buildCircle,
  [SHAPES.RECT]: buildRectangle,
  [SHAPES.RREC]: buildRoundedRectangle
};
const BATCH_POOL = [];
const DRAW_CALL_POOL = [];

export { BATCH_POOL, DRAW_CALL_POOL, FILL_COMMANDS };
//# sourceMappingURL=index.mjs.map
