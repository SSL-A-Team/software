// Composables
import { provideDefaults } from "../../composables/defaults.mjs"; // Utilities
import { toRefs } from 'vue';
import { genericComponent, propsFactory } from "../../util/index.mjs"; // Types
export const makeDefaultsProviderProps = propsFactory({
  defaults: Object,
  disabled: Boolean,
  reset: [Number, String],
  root: Boolean,
  scoped: Boolean
}, 'v-defaults-provider');
export const VDefaultsProvider = genericComponent(false)({
  name: 'VDefaultsProvider',
  props: makeDefaultsProviderProps(),
  setup(props, _ref) {
    let {
      slots
    } = _ref;
    const {
      defaults,
      disabled,
      reset,
      root,
      scoped
    } = toRefs(props);
    provideDefaults(defaults, {
      reset,
      root,
      scoped,
      disabled
    });
    return () => slots.default?.();
  }
});
//# sourceMappingURL=VDefaultsProvider.mjs.map