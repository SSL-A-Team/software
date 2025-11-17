import { Filter } from '@pixi/core';
import fragment from './fxaa.mjs';
import vertex from './fxaa2.mjs';

class FXAAFilter extends Filter {
  constructor() {
    super(vertex, fragment);
  }
}

export { FXAAFilter };
//# sourceMappingURL=FXAAFilter.mjs.map
