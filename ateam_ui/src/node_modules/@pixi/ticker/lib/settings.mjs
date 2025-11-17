import { settings } from '@pixi/settings';
export { settings } from '@pixi/settings';
import { deprecation } from '@pixi/utils';
import { Ticker } from './Ticker.mjs';

Object.defineProperties(settings, {
  TARGET_FPMS: {
    get() {
      return Ticker.targetFPMS;
    },
    set(value) {
      deprecation("7.1.0", "settings.TARGET_FPMS is deprecated, use Ticker.targetFPMS");
      Ticker.targetFPMS = value;
    }
  }
});
//# sourceMappingURL=settings.mjs.map
