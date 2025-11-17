import { settings, utils } from '@pixi/core';
export { settings } from '@pixi/core';
import { Container } from './Container.mjs';

Object.defineProperties(settings, {
  SORTABLE_CHILDREN: {
    get() {
      return Container.defaultSortableChildren;
    },
    set(value) {
      utils.deprecation("7.1.0", "settings.SORTABLE_CHILDREN is deprecated, use Container.defaultSortableChildren");
      Container.defaultSortableChildren = value;
    }
  }
});
//# sourceMappingURL=settings.mjs.map
