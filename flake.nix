{
  description = "Make your rover autonomous";
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url =
      "github:NixOS/nixpkgs/86388ba6881aae014fb945ede340083fef0fd637";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let pkgs = import nixpkgs {
        inherit system;
        config.permittedInsecurePackages = [
          "tensorflow-lite-2.5.0"
          "tensorflow-lite-2.10.0"
        ];
      };
      in rec {
        packages.librealsense = pkgs.librealsense.overrideAttrs
          (finalAttrs: previousAttrs: {
            postInstall = ''
              substituteInPlace $out/lib/cmake/realsense2/realsense2Targets.cmake \
                --replace "\''${_IMPORT_PREFIX}/include" "$dev/include"
            '';
          });
        packages.tflite = pkgs.tensorflow-lite.overrideAttrs
          (finalAttrs: previousAttrs: {
            version = "2.10.0";
            src = pkgs.fetchFromGitHub {
              owner = "tensorflow";
              repo = "tensorflow";
              rev = "v2.10.0";
              sha256 = "sha256-Y6cujiMoQXKQlsLBr7d0T278ltdd00IfsTRycJbRVN4=";
            };
            nativeBuildInputs = [ pkgs.cmake ];
            patches = [];
            postPatch = "";
            dontConfigure = false;
            configurePhase = ''
              cmake -S ./tensorflow/lite -B build -DTFLITE_ENABLE_INSTALL=ON \
                -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON \
                -DSYSTEM_FARMHASH=ON \
                -Dabsl_DIR=${previousAttrs.abseil-cpp.src}/lib/cmake/absl \
                -DEigen3_DIR=${previousAttrs.tflite-eigen}/share/eigen3/cmake \
                -DFlatBuffers_DIR=${previousAttrs.flatbuffers.src}/lib/cmake/flatbuffers \
                -DNEON_2_SSE_DIR=${previousAttrs.neon-2-sse-src}/lib/cmake/NEON_2_SSE \
                -Dcpuinfo_DIR=${previousAttrs.cpuinfo-src}/share/cpuinfo \
                -Druy_DIR=${previousAttrs.ruy-src}/lib/cmake/ruy
            '';
            preBuild = ''
              cd build
            '';
            installPhase = ''
              mkdir -p "$out/bin"
              cp -r ../build $out/bin
            '';
          });
        packages.michi = with pkgs;
          stdenv.mkDerivation {
            name = "michi";
            src = self;
            nativeBuildInputs = [ cmake gcc13 pkgconfig ];
            buildInputs = [
              packages.librealsense.dev
              pcl
              boost.dev
              opencv
              glfw
              libGLU.dev
              spdlog.dev
              asio
              gtest.dev
              packages.tflite
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
      });
}
