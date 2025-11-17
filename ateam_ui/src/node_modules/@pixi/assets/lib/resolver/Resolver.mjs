import { utils } from '@pixi/core';
import { convertToList } from '../utils/convertToList.mjs';
import { createStringVariations } from '../utils/createStringVariations.mjs';
import { isSingleItem } from '../utils/isSingleItem.mjs';

class Resolver {
  constructor() {
    this._defaultBundleIdentifierOptions = {
      connector: "-",
      createBundleAssetId: (bundleId, assetId) => `${bundleId}${this._bundleIdConnector}${assetId}`,
      extractAssetIdFromBundle: (bundleId, assetBundleId) => assetBundleId.replace(`${bundleId}${this._bundleIdConnector}`, "")
    };
    this._bundleIdConnector = this._defaultBundleIdentifierOptions.connector;
    this._createBundleAssetId = this._defaultBundleIdentifierOptions.createBundleAssetId;
    this._extractAssetIdFromBundle = this._defaultBundleIdentifierOptions.extractAssetIdFromBundle;
    this._assetMap = {};
    this._preferredOrder = [];
    this._parsers = [];
    this._resolverHash = {};
    this._bundles = {};
  }
  setBundleIdentifier(bundleIdentifier) {
    this._bundleIdConnector = bundleIdentifier.connector ?? this._bundleIdConnector;
    this._createBundleAssetId = bundleIdentifier.createBundleAssetId ?? this._createBundleAssetId;
    this._extractAssetIdFromBundle = bundleIdentifier.extractAssetIdFromBundle ?? this._extractAssetIdFromBundle;
    if (this._extractAssetIdFromBundle("foo", this._createBundleAssetId("foo", "bar")) !== "bar") {
      throw new Error("[Resolver] GenerateBundleAssetId are not working correctly");
    }
  }
  prefer(...preferOrders) {
    preferOrders.forEach((prefer) => {
      this._preferredOrder.push(prefer);
      if (!prefer.priority) {
        prefer.priority = Object.keys(prefer.params);
      }
    });
    this._resolverHash = {};
  }
  set basePath(basePath) {
    this._basePath = basePath;
  }
  get basePath() {
    return this._basePath;
  }
  set rootPath(rootPath) {
    this._rootPath = rootPath;
  }
  get rootPath() {
    return this._rootPath;
  }
  get parsers() {
    return this._parsers;
  }
  reset() {
    this.setBundleIdentifier(this._defaultBundleIdentifierOptions);
    this._assetMap = {};
    this._preferredOrder = [];
    this._resolverHash = {};
    this._rootPath = null;
    this._basePath = null;
    this._manifest = null;
    this._bundles = {};
    this._defaultSearchParams = null;
  }
  setDefaultSearchParams(searchParams) {
    if (typeof searchParams === "string") {
      this._defaultSearchParams = searchParams;
    } else {
      const queryValues = searchParams;
      this._defaultSearchParams = Object.keys(queryValues).map((key) => `${encodeURIComponent(key)}=${encodeURIComponent(queryValues[key])}`).join("&");
    }
  }
  addManifest(manifest) {
    if (this._manifest) {
      console.warn("[Resolver] Manifest already exists, this will be overwritten");
    }
    this._manifest = manifest;
    manifest.bundles.forEach((bundle) => {
      this.addBundle(bundle.name, bundle.assets);
    });
  }
  addBundle(bundleId, assets) {
    const assetNames = [];
    if (Array.isArray(assets)) {
      assets.forEach((asset) => {
        if (typeof asset.name === "string") {
          const bundleAssetId = this._createBundleAssetId(bundleId, asset.name);
          assetNames.push(bundleAssetId);
          this.add([asset.name, bundleAssetId], asset.srcs, asset.data);
        } else {
          const bundleIds = asset.name.map((name) => this._createBundleAssetId(bundleId, name));
          bundleIds.forEach((bundleId2) => {
            assetNames.push(bundleId2);
          });
          this.add([...asset.name, ...bundleIds], asset.srcs);
        }
      });
    } else {
      Object.keys(assets).forEach((key) => {
        assetNames.push(this._createBundleAssetId(bundleId, key));
        this.add([key, this._createBundleAssetId(bundleId, key)], assets[key]);
      });
    }
    this._bundles[bundleId] = assetNames;
  }
  add(keysIn, assetsIn, data) {
    const keys = convertToList(keysIn);
    keys.forEach((key) => {
      if (this.hasKey(key)) {
        console.warn(`[Resolver] already has key: ${key} overwriting`);
      }
    });
    if (!Array.isArray(assetsIn)) {
      if (typeof assetsIn === "string") {
        assetsIn = createStringVariations(assetsIn);
      } else {
        assetsIn = [assetsIn];
      }
    }
    const assetMap = assetsIn.map((asset) => {
      let formattedAsset = asset;
      if (typeof asset === "string") {
        let parsed = false;
        for (let i = 0; i < this._parsers.length; i++) {
          const parser = this._parsers[i];
          if (parser.test(asset)) {
            formattedAsset = parser.parse(asset);
            parsed = true;
            break;
          }
        }
        if (!parsed) {
          formattedAsset = {
            src: asset
          };
        }
      }
      if (!formattedAsset.format) {
        formattedAsset.format = formattedAsset.src.split(".").pop();
      }
      if (!formattedAsset.alias) {
        formattedAsset.alias = keys;
      }
      if (this._basePath || this._rootPath) {
        formattedAsset.src = utils.path.toAbsolute(formattedAsset.src, this._basePath, this._rootPath);
      }
      formattedAsset.src = this._appendDefaultSearchParams(formattedAsset.src);
      formattedAsset.data = formattedAsset.data ?? data;
      return formattedAsset;
    });
    keys.forEach((key) => {
      this._assetMap[key] = assetMap;
    });
  }
  resolveBundle(bundleIds) {
    const singleAsset = isSingleItem(bundleIds);
    bundleIds = convertToList(bundleIds);
    const out = {};
    bundleIds.forEach((bundleId) => {
      const assetNames = this._bundles[bundleId];
      if (assetNames) {
        const results = this.resolve(assetNames);
        const assets = {};
        for (const key in results) {
          const asset = results[key];
          assets[this._extractAssetIdFromBundle(bundleId, key)] = asset;
        }
        out[bundleId] = assets;
      }
    });
    return singleAsset ? out[bundleIds[0]] : out;
  }
  resolveUrl(key) {
    const result = this.resolve(key);
    if (typeof key !== "string") {
      const out = {};
      for (const i in result) {
        out[i] = result[i].src;
      }
      return out;
    }
    return result.src;
  }
  resolve(keys) {
    const singleAsset = isSingleItem(keys);
    keys = convertToList(keys);
    const result = {};
    keys.forEach((key) => {
      if (!this._resolverHash[key]) {
        if (this._assetMap[key]) {
          let assets = this._assetMap[key];
          const preferredOrder = this._getPreferredOrder(assets);
          const bestAsset = assets[0];
          preferredOrder?.priority.forEach((priorityKey) => {
            preferredOrder.params[priorityKey].forEach((value) => {
              const filteredAssets = assets.filter((asset) => {
                if (asset[priorityKey]) {
                  return asset[priorityKey] === value;
                }
                return false;
              });
              if (filteredAssets.length) {
                assets = filteredAssets;
              }
            });
          });
          this._resolverHash[key] = assets[0] ?? bestAsset;
        } else {
          let src = key;
          if (this._basePath || this._rootPath) {
            src = utils.path.toAbsolute(src, this._basePath, this._rootPath);
          }
          src = this._appendDefaultSearchParams(src);
          this._resolverHash[key] = {
            src
          };
        }
      }
      result[key] = this._resolverHash[key];
    });
    return singleAsset ? result[keys[0]] : result;
  }
  hasKey(key) {
    return !!this._assetMap[key];
  }
  hasBundle(key) {
    return !!this._bundles[key];
  }
  _getPreferredOrder(assets) {
    for (let i = 0; i < assets.length; i++) {
      const asset = assets[0];
      const preferred = this._preferredOrder.find((preference) => preference.params.format.includes(asset.format));
      if (preferred) {
        return preferred;
      }
    }
    return this._preferredOrder[0];
  }
  _appendDefaultSearchParams(url) {
    if (!this._defaultSearchParams)
      return url;
    const paramConnector = /\?/.test(url) ? "&" : "?";
    return `${url}${paramConnector}${this._defaultSearchParams}`;
  }
}

export { Resolver };
//# sourceMappingURL=Resolver.mjs.map
