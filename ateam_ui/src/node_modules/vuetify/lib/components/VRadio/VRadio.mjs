import { createVNode as _createVNode, mergeProps as _mergeProps, resolveDirective as _resolveDirective } from "vue";
// Components
import { makeSelectionControlProps, VSelectionControl } from "../VSelectionControl/VSelectionControl.mjs"; // Utilities
import { genericComponent, propsFactory, useRender } from "../../util/index.mjs"; // Types
export const makeVRadioProps = propsFactory({
  ...makeSelectionControlProps({
    falseIcon: '$radioOff',
    trueIcon: '$radioOn'
  })
}, 'v-radio');
export const VRadio = genericComponent()({
  name: 'VRadio',
  props: makeVRadioProps(),
  setup(props, _ref) {
    let {
      slots
    } = _ref;
    useRender(() => _createVNode(VSelectionControl, _mergeProps(props, {
      "class": ['v-radio', props.class],
      "style": props.style,
      "type": "radio"
    }), slots));
    return {};
  }
});
//# sourceMappingURL=VRadio.mjs.map