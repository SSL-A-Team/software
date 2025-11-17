import { createVNode as _createVNode } from "vue";
// Composables
import { makeComponentProps } from "../../composables/component.mjs";
import { useResizeObserver } from "../../composables/resizeObserver.mjs";
import { useToggleScope } from "../../composables/toggleScope.mjs"; // Utilities
import { genericComponent, propsFactory, useRender } from "../../util/index.mjs";
import { onUpdated, watch } from 'vue';
export const makeVVirtualScrollItemProps = propsFactory({
  dynamicHeight: Boolean,
  ...makeComponentProps()
}, 'v-virtual-scroll-item');
export const VVirtualScrollItem = genericComponent()({
  name: 'VVirtualScrollItem',
  props: makeVVirtualScrollItemProps(),
  emits: {
    'update:height': height => true
  },
  setup(props, _ref) {
    let {
      emit,
      slots
    } = _ref;
    const {
      resizeRef,
      contentRect
    } = useResizeObserver();
    useToggleScope(() => props.dynamicHeight, () => {
      watch(() => contentRect.value?.height, height => {
        if (height != null) emit('update:height', height);
      });
    });
    function updateHeight() {
      if (props.dynamicHeight && contentRect.value) {
        emit('update:height', contentRect.value.height);
      }
    }
    onUpdated(updateHeight);
    useRender(() => _createVNode("div", {
      "ref": props.dynamicHeight ? resizeRef : undefined,
      "class": ['v-virtual-scroll__item', props.class],
      "style": props.style
    }, [slots.default?.()]));
  }
});
//# sourceMappingURL=VVirtualScrollItem.mjs.map