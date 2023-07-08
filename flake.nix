{
  description = "Make your rover autonomous";
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/86388ba6881aae014fb945ede340083fef0fd637";
  };

  outputs = { self, nixpkgs, flake-utils}: 
   flake-utils.lib.eachDefaultSystem (system:
  let pkgs = nixpkgs.legacyPackages.${system}; in rec {
    packages.librealsense = pkgs.librealsense.overrideAttrs(finalAttrs: previousAttrs: {
      postInstall = ''
        substituteInPlace $out/lib/cmake/realsense2/realsense2Targets.cmake \
          --replace "\''${_IMPORT_PREFIX}/include" "$dev/include"
          '';
    });
    packages.michi = with pkgs; stdenv.mkDerivation {
      name = "michi";
      src = self;
      nativeBuildInputs = [ cmake gcc pkgconfig ];
      buildInputs = [
        packages.librealsense.dev
        pcl
        boost.dev
        opencv
        glfw
        libGLU.dev
        gtest.dev
        glog
      ];
      configurePhase = ''
      cmake -S . -B build
      '';
      preBuild = ''
      cd build
      '';
      installPhase = ''
      mkdir -p "$out/bin"
      cp -r ../build $out/bin
      '';
    };
    defaultPackage = packages.michi;
    }
  );
}
