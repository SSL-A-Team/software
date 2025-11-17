import isMobileJs from 'ismobilejs';

const isMobileCall = isMobileJs.default ?? isMobileJs;
const isMobile = isMobileCall(globalThis.navigator);

export { isMobile };
//# sourceMappingURL=isMobile.mjs.map
