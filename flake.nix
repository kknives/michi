{
  description = "Make your rover autonomous";
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url =
      "github:NixOS/nixpkgs/86388ba6881aae014fb945ede340083fef0fd637";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let pkgs = nixpkgs.legacyPackages.${system};
      in rec {
        packages.librealsense = pkgs.librealsense.overrideAttrs
          (finalAttrs: previousAttrs: {
            postInstall = ''
              substituteInPlace $out/lib/cmake/realsense2/realsense2Targets.cmake \
                --replace "\''${_IMPORT_PREFIX}/include" "$dev/include"
            '';
          });
          packages.mavlink_c = with pkgs;
            stdenv.mkDerivation {
              name = "mavlink_c_library_v2";
              src = fetchFromGitHub {
                owner = "mavlink";
                repo = "c_library_v2";
                rev = "494fd857a34267d01c2d3c2d601ecfa651f73489";
                sha256 = "sha256-FiuD9G+1sSYfBFpTCw6c5mnpFbDkZJwYFYtL3o1ujAo=";
              };
              nativeBuildInputs = [ copyPkgconfigItems ];
              pkgconfigItems = [
                (makePkgconfigItem rec {
                  name = "mavlink_c";
                  version = "2";
                  cflags = [ "-I${variables.includedir}/mavlink"];
                  variables = rec {
                    prefix = "${placeholder "out"}";
                    includedir = "${prefix}/include";
                  };
                })
              ];
              dontBuild = true;
              installPhase = ''
              runHook preInstall
              mkdir -p $out/include/mavlink
              cp -R standard common minimal $out/include/mavlink
              cp *.h $out/include/mavlink
              runHook postInstall
              '';
            };
        packages.michi = with pkgs;
          stdenv.mkDerivation {
            name = "michi";
            src = self;
            nativeBuildInputs = [ cmake gcc13 pkgconfig ];
            buildInputs = [
              packages.librealsense.dev
              eigen
              pcl
              boost.dev
              opencv
              glfw
              libGLU.dev
              spdlog.dev
              asio
              gtest.dev
              onnxruntime.dev
              packages.mavlink_c
              argparse
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
