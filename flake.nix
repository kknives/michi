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
        packages.gz_cmake = with pkgs;
          stdenv.mkDerivation {
            name = "gz_cmake";
            src = fetchFromGitHub {
              owner = "gazebosim";
              repo = "gz-cmake";
              rev = "459ea347de71d5a9cd0ef2cf13fe14e6421ecb6e";
              sha256 = "sha256-cmEdtGQ2h3eelRbyr9MLCrkI/phoqaCSA7wv2fW+ylo=";
            };
            nativeBuildInputs = [cmake];
            prePatch = ''
              substituteInPlace config/gz-cmake.pc.in \
                --replace '$'{prefix}/@CMAKE_INSTALL_LIBDIR@ @CMAKE_INSTALL_FULL_LIBDIR@ \
                --replace '$'{prefix}/@CMAKE_INSTALL_INCLUDEDIR@ @CMAKE_INSTALL_FULL_INCLUDEDIR@
                '';
            configurePhase = ''
                mkdir build && cd build
                cmake .. -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out
              '';

              installPhase = ''
                make install
              '';

              meta = {
                description = "gz-cmake3";
              };
          };
          packages.gz_utils = with pkgs;
          stdenv.mkDerivation {
            name = "gz_utils";
            src = fetchFromGitHub {
              owner = "gazebosim";
              repo = "gz-utils";
              rev = "8f17f54c48ac21382f9f13776e66906046cc3d94";
              sha256 = "sha256-osY+q+H7F05gcLrpyMGeLsddh2nevG4lZsFeyeZWdaY=";
            };
            nativeBuildInputs = [cmake];
            buildInputs = [packages.gz_cmake];
            configurePhase = ''
                mkdir build && cd build
                cmake .. -DCMAKE_INSTALL_PREFIX=$out -DBUILD_TESTING=OFF
              '';

              installPhase = ''
              runHook preInstall
                cmake --install .
              runHook postInstall
              '';
              meta = {
                description = "gz-utils2";
              };
          };
        packages.gz_math = with pkgs;
          stdenv.mkDerivation {
            name = "gz_math";
            src = fetchFromGitHub {
              owner = "gazebosim";
              repo = "gz-math";
              rev = "39e48c1388e30a1eac101bc89d34937a394d7d95";
              sha256 = "sha256-Gj69j3PH4AlSMvzd3OjPF5wmQ0PfDKDtSH3CIgdFxBg=";
            };
            nativeBuildInputs = [cmake pkgconfig];
            buildInputs = [eigen packages.gz_cmake packages.gz_utils];
            configurePhase = ''
                mkdir build && cd build
                cmake .. -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out \
                -DBUILD_TESTING=OFF
              '';

              installPhase = ''
                make install
              '';
              meta = {
                description = "gz-math7";
              };
          };
        packages.gz_msgs = with pkgs;
          stdenv.mkDerivation {
            name = "gz_msgs";
            src = fetchFromGitHub {
              owner = "gazebosim";
              repo = "gz-msgs";
              rev = "0472ba0bb5fe39d8a14499155c68746109d9acf7";
              sha256 = "sha256-wRbvGJAjwUD4YMlvgP70DytKGrPEhhxtIUcaLPkZ68I=";
            };
            nativeBuildInputs = [cmake pkgconfig];
            buildInputs = [protobuf packages.gz_math packages.gz_cmake packages.gz_utils python3 tinyxml-2];
            configurePhase = ''
                mkdir build && cd build
                cmake .. -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX=$out \
                -DBUILD_TESTING=OFF
              '';

              installPhase = ''
                make install
              '';
              meta = {
                description = "gz-msgs10";
              };
          };
        packages.gz_transport = with pkgs;
          stdenv.mkDerivation {
            name = "gz_transport";
            src = fetchFromGitHub {
              owner = "gazebosim";
              repo = "gz-transport";
              rev = "cfb80d25904920fe86ae8b1ca50b8fe46788d00f";
              sha256 = "sha256-+jEkBeXujnChYemWt+XwCE8CqLpMpnc7nP4vl8C3kOQ=";
            };
            nativeBuildInputs = [cmake pkgconfig];
            buildInputs = [
              protobuf
              libuuid.dev
              zeromq
              tinyxml-2
              python3
              sqlite
              cppzmq
              packages.gz_msgs
              packages.gz_math
              packages.gz_cmake
              packages.gz_utils
            ];
              meta = {
                description = "gz-transport13";
              };
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
              protobuf
              libuuid.dev
              zeromq
              cppzmq
              tinyxml-2
              libsodium
              packages.gz_transport
              packages.gz_cmake
              packages.gz_utils
              packages.gz_math
              packages.gz_msgs
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
