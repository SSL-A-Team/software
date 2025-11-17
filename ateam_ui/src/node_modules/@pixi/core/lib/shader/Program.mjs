import { PRECISION } from '@pixi/constants';
import { ProgramCache, isMobile } from '@pixi/utils';
import defaultFragment from './defaultProgram.mjs';
import defaultVertex from './defaultProgram2.mjs';
import './utils/index.mjs';
import { setPrecision } from './utils/setPrecision.mjs';
import { getMaxFragmentPrecision } from './utils/getMaxFragmentPrecision.mjs';

let UID = 0;
const nameCache = {};
const _Program = class {
  constructor(vertexSrc, fragmentSrc, name = "pixi-shader", extra = {}) {
    this.extra = {};
    this.id = UID++;
    this.vertexSrc = vertexSrc || _Program.defaultVertexSrc;
    this.fragmentSrc = fragmentSrc || _Program.defaultFragmentSrc;
    this.vertexSrc = this.vertexSrc.trim();
    this.fragmentSrc = this.fragmentSrc.trim();
    this.extra = extra;
    if (this.vertexSrc.substring(0, 8) !== "#version") {
      name = name.replace(/\s+/g, "-");
      if (nameCache[name]) {
        nameCache[name]++;
        name += `-${nameCache[name]}`;
      } else {
        nameCache[name] = 1;
      }
      this.vertexSrc = `#define SHADER_NAME ${name}
${this.vertexSrc}`;
      this.fragmentSrc = `#define SHADER_NAME ${name}
${this.fragmentSrc}`;
      this.vertexSrc = setPrecision(this.vertexSrc, _Program.defaultVertexPrecision, PRECISION.HIGH);
      this.fragmentSrc = setPrecision(this.fragmentSrc, _Program.defaultFragmentPrecision, getMaxFragmentPrecision());
    }
    this.glPrograms = {};
    this.syncUniforms = null;
  }
  static get defaultVertexSrc() {
    return defaultVertex;
  }
  static get defaultFragmentSrc() {
    return defaultFragment;
  }
  static from(vertexSrc, fragmentSrc, name) {
    const key = vertexSrc + fragmentSrc;
    let program = ProgramCache[key];
    if (!program) {
      ProgramCache[key] = program = new _Program(vertexSrc, fragmentSrc, name);
    }
    return program;
  }
};
let Program = _Program;
Program.defaultVertexPrecision = PRECISION.HIGH;
Program.defaultFragmentPrecision = isMobile.apple.device ? PRECISION.HIGH : PRECISION.MEDIUM;

export { Program };
//# sourceMappingURL=Program.mjs.map
