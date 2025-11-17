const _BoundingBox = class {
  constructor(left, top, right, bottom) {
    this.left = left;
    this.top = top;
    this.right = right;
    this.bottom = bottom;
  }
  get width() {
    return this.right - this.left;
  }
  get height() {
    return this.bottom - this.top;
  }
  isEmpty() {
    return this.left === this.right || this.top === this.bottom;
  }
};
let BoundingBox = _BoundingBox;
BoundingBox.EMPTY = new _BoundingBox(0, 0, 0, 0);

export { BoundingBox };
//# sourceMappingURL=BoundingBox.mjs.map
