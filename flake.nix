{
  description = "Area I ReMAP Package";

  inputs = {
    nixpkgs.url = github:NixOS/nixpkgs/nixos-22.05;
    flake-utils.url = github:numtide/flake-utils;

    nix-ros.url = "github:clearpathrobotics/nix-ros?ref=refs/tags/20221020-1";

  };

  outputs =
    { self
    , nixpkgs
    , flake-utils
    , nix-ros
    , ...
    }:

  let
    overlays =
      nix-ros.base-overlays
      ++ [
        (final: prev: {

          remap = prev.callPackage
            ({ stdenv, cmake, boost174, ai-geometry, terrain-manager, ai-test, aicomm, aivbus, ai-pathplanning }:
              stdenv.mkDerivation
              {
                pname = "remap";
                version = "3.6.0";
                meta.owner = "areai";
                src = remap;
                nativeBuildInputs = [ cmake ];
              }
            ){ };

        })
      ];
  in
    { inherit overlays; } // flake-utils.lib.eachDefaultSystem (system: rec
    {
      legacyPackages = import nixpkgs { inherit system; inherit overlays; };

      defaultPackage = legacyPackages.remap;
    });
  # end in
}

