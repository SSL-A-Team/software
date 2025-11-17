import { settings, utils } from '@pixi/core';
export { settings } from '@pixi/core';
import { BasePrepare } from './BasePrepare.mjs';

Object.defineProperties(settings, {
  UPLOADS_PER_FRAME: {
    get() {
      return BasePrepare.uploadsPerFrame;
    },
    set(value) {
      utils.deprecation("7.1.0", "settings.UPLOADS_PER_FRAME is deprecated, use prepare.BasePrepare.uploadsPerFrame");
      BasePrepare.uploadsPerFrame = value;
    }
  }
});
//# sourceMappingURL=settings.mjs.map
