import { createVNode as _createVNode } from "vue";
// Composables
import { makeComponentProps } from "../../composables/component.mjs";
import { makeTagProps } from "../../composables/tag.mjs";
import { useTextColor } from "../../composables/color.mjs"; // Utilities
import { toRef } from 'vue';
import { genericComponent, propsFactory, useRender } from "../../util/index.mjs";
export const makeVListSubheaderProps = propsFactory({
  color: String,
  inset: Boolean,
  sticky: Boolean,
  title: String,
  ...makeComponentProps(),
  ...makeTagProps()
}, 'v-list-subheader');
export const VListSubheader = genericComponent()({
  name: 'VListSubheader',
  props: makeVListSubheaderProps(),
  setup(props, _ref) {
    let {
      slots
    } = _ref;
    const {
      textColorClasses,
      textColorStyles
    } = useTextColor(toRef(props, 'color'));
    useRender(() => {
      const hasText = !!(slots.default || props.title);
      return _createVNode(props.tag, {
        "class": ['v-list-subheader', {
          'v-list-subheader--inset': props.inset,
          'v-list-subheader--sticky': props.sticky
        }, textColorClasses.value, props.class],
        "style": [{
          textColorStyles
        }, props.style]
      }, {
        default: () => [hasText && _createVNode("div", {
          "class": "v-list-subheader__text"
        }, [slots.default?.() ?? props.title])]
      });
    });
    return {};
  }
});
//# sourceMappingURL=VListSubheader.mjs.map