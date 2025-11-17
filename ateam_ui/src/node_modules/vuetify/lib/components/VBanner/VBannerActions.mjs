import { createVNode as _createVNode } from "vue";
// Composables
import { makeComponentProps } from "../../composables/component.mjs";
import { provideDefaults } from "../../composables/defaults.mjs"; // Utility
import { genericComponent, propsFactory, useRender } from "../../util/index.mjs";
export const makeVBannerActionsProps = propsFactory({
  color: String,
  density: String,
  ...makeComponentProps()
}, 'v-banner-actions');
export const VBannerActions = genericComponent()({
  name: 'VBannerActions',
  props: makeVBannerActionsProps(),
  setup(props, _ref) {
    let {
      slots
    } = _ref;
    provideDefaults({
      VBtn: {
        color: props.color,
        density: props.density,
        variant: 'text'
      }
    });
    useRender(() => _createVNode("div", {
      "class": ['v-banner-actions', props.class],
      "style": props.style
    }, [slots.default?.()]));
    return {};
  }
});
//# sourceMappingURL=VBannerActions.mjs.map