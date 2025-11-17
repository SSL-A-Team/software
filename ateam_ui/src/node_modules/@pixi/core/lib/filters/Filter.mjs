import { MSAA_QUALITY } from '@pixi/constants';
import { Program } from '../shader/Program.mjs';
import { Shader } from '../shader/Shader.mjs';
import { State } from '../state/State.mjs';
import defaultFragment from './defaultFilter.mjs';
import defaultVertex from './defaultFilter2.mjs';

const _Filter = class extends Shader {
  constructor(vertexSrc, fragmentSrc, uniforms) {
    const program = Program.from(vertexSrc || _Filter.defaultVertexSrc, fragmentSrc || _Filter.defaultFragmentSrc);
    super(program, uniforms);
    this.padding = 0;
    this.resolution = _Filter.defaultResolution;
    this.multisample = _Filter.defaultMultisample;
    this.enabled = true;
    this.autoFit = true;
    this.state = new State();
  }
  apply(filterManager, input, output, clearMode, _currentState) {
    filterManager.applyFilter(this, input, output, clearMode);
  }
  get blendMode() {
    return this.state.blendMode;
  }
  set blendMode(value) {
    this.state.blendMode = value;
  }
  get resolution() {
    return this._resolution;
  }
  set resolution(value) {
    this._resolution = value;
  }
  static get defaultVertexSrc() {
    return defaultVertex;
  }
  static get defaultFragmentSrc() {
    return defaultFragment;
  }
};
let Filter = _Filter;
Filter.defaultResolution = 1;
Filter.defaultMultisample = MSAA_QUALITY.NONE;

export { Filter };
//# sourceMappingURL=Filter.mjs.map
