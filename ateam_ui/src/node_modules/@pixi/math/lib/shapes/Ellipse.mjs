import { SHAPES } from '../const.mjs';
import { Rectangle } from './Rectangle.mjs';

class Ellipse {
  constructor(x = 0, y = 0, halfWidth = 0, halfHeight = 0) {
    this.x = x;
    this.y = y;
    this.width = halfWidth;
    this.height = halfHeight;
    this.type = SHAPES.ELIP;
  }
  clone() {
    return new Ellipse(this.x, this.y, this.width, this.height);
  }
  contains(x, y) {
    if (this.width <= 0 || this.height <= 0) {
      return false;
    }
    let normx = (x - this.x) / this.width;
    let normy = (y - this.y) / this.height;
    normx *= normx;
    normy *= normy;
    return normx + normy <= 1;
  }
  getBounds() {
    return new Rectangle(this.x - this.width, this.y - this.height, this.width, this.height);
  }
  toString() {
    return `[@pixi/math:Ellipse x=${this.x} y=${this.y} width=${this.width} height=${this.height}]`;
  }
}

export { Ellipse };
//# sourceMappingURL=Ellipse.mjs.map
