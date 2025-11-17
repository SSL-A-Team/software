import { createVNode as _createVNode } from "vue";
// Composables
import { makeComponentProps } from "../../composables/component.mjs";
import { makeVariantProps } from "../../composables/variant.mjs";
import { provideDefaults } from "../../composables/defaults.mjs"; // Utilities
import { genericComponent, propsFactory, useRender } from "../../util/index.mjs";
import { toRef } from 'vue';
export const makeVToolbarItemsProps = propsFactory({
  ...makeComponentProps(),
  ...makeVariantProps({
    variant: 'text'
  })
}, 'v-toolbar-items');
export const VToolbarItems = genericComponent()({
  name: 'VToolbarItems',
  props: makeVToolbarItemsProps(),
  setup(props, _ref) {
    let {
      slots
    } = _ref;
    provideDefaults({
      VBtn: {
        color: toRef(props, 'color'),
        height: 'inherit',
        variant: toRef(props, 'variant')
      }
    });
    useRender(() => _createVNode("div", {
      "class": ['v-toolbar-items', props.class],
      "style": props.style
    }, [slots.default?.()]));
    return {};
  }
});
//# sourceMappingURL=VToolbarItems.mjs.map