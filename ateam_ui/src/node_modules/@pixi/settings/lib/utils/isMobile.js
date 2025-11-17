'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var isMobileJs = require('ismobilejs');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var isMobileJs__default = /*#__PURE__*/_interopDefaultLegacy(isMobileJs);

const isMobileCall = isMobileJs__default["default"].default ?? isMobileJs__default["default"];
const isMobile = isMobileCall(globalThis.navigator);

exports.isMobile = isMobile;
//# sourceMappingURL=isMobile.js.map
