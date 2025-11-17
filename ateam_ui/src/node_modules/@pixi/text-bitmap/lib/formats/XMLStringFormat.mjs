import { settings } from '@pixi/core';
import { XMLFormat } from './XMLFormat.mjs';

class XMLStringFormat {
  static test(data) {
    if (typeof data === "string" && data.includes("<font>")) {
      return XMLFormat.test(settings.ADAPTER.parseXML(data));
    }
    return false;
  }
  static parse(xmlTxt) {
    return XMLFormat.parse(settings.ADAPTER.parseXML(xmlTxt));
  }
}

export { XMLStringFormat };
//# sourceMappingURL=XMLStringFormat.mjs.map
