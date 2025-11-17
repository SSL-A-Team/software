import { TYPES } from '@pixi/constants';

class Attribute {
  constructor(buffer, size = 0, normalized = false, type = TYPES.FLOAT, stride, start, instance, divisor = 1) {
    this.buffer = buffer;
    this.size = size;
    this.normalized = normalized;
    this.type = type;
    this.stride = stride;
    this.start = start;
    this.instance = instance;
    this.divisor = divisor;
  }
  destroy() {
    this.buffer = null;
  }
  static from(buffer, size, normalized, type, stride) {
    return new Attribute(buffer, size, normalized, type, stride);
  }
}

export { Attribute };
//# sourceMappingURL=Attribute.mjs.map
