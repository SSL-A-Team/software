import { Point } from '@pixi/core';
import { DisplayObject } from '@pixi/display';

DisplayObject.prototype.getGlobalPosition = function getGlobalPosition(point = new Point(), skipUpdate = false) {
  if (this.parent) {
    this.parent.toGlobal(this.position, point, skipUpdate);
  } else {
    point.x = this.position.x;
    point.y = this.position.y;
  }
  return point;
};
//# sourceMappingURL=index.mjs.map
