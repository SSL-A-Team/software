import { GC_MODES } from '@pixi/constants';
import { ExtensionType, extensions } from '@pixi/extensions';

const _TextureGCSystem = class {
  constructor(renderer) {
    this.renderer = renderer;
    this.count = 0;
    this.checkCount = 0;
    this.maxIdle = _TextureGCSystem.defaultMaxIdle;
    this.checkCountMax = _TextureGCSystem.defaultCheckCountMax;
    this.mode = _TextureGCSystem.defaultMode;
  }
  postrender() {
    if (!this.renderer.objectRenderer.renderingToScreen) {
      return;
    }
    this.count++;
    if (this.mode === GC_MODES.MANUAL) {
      return;
    }
    this.checkCount++;
    if (this.checkCount > this.checkCountMax) {
      this.checkCount = 0;
      this.run();
    }
  }
  run() {
    const tm = this.renderer.texture;
    const managedTextures = tm.managedTextures;
    let wasRemoved = false;
    for (let i = 0; i < managedTextures.length; i++) {
      const texture = managedTextures[i];
      if (!texture.framebuffer && this.count - texture.touched > this.maxIdle) {
        tm.destroyTexture(texture, true);
        managedTextures[i] = null;
        wasRemoved = true;
      }
    }
    if (wasRemoved) {
      let j = 0;
      for (let i = 0; i < managedTextures.length; i++) {
        if (managedTextures[i] !== null) {
          managedTextures[j++] = managedTextures[i];
        }
      }
      managedTextures.length = j;
    }
  }
  unload(displayObject) {
    const tm = this.renderer.texture;
    const texture = displayObject._texture;
    if (texture && !texture.framebuffer) {
      tm.destroyTexture(texture);
    }
    for (let i = displayObject.children.length - 1; i >= 0; i--) {
      this.unload(displayObject.children[i]);
    }
  }
  destroy() {
    this.renderer = null;
  }
};
let TextureGCSystem = _TextureGCSystem;
TextureGCSystem.defaultMode = GC_MODES.AUTO;
TextureGCSystem.defaultMaxIdle = 60 * 60;
TextureGCSystem.defaultCheckCountMax = 60 * 10;
TextureGCSystem.extension = {
  type: ExtensionType.RendererSystem,
  name: "textureGC"
};
extensions.add(TextureGCSystem);

export { TextureGCSystem };
//# sourceMappingURL=TextureGCSystem.mjs.map
