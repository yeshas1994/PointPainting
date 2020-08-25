Make sure to install Magnum as 64-bit.
https://github.com/microsoft/vcpkg/issues/1254

vcpkg install magnum --triplet x64-windows
vcpkg install opencv4[world] --triplet x64-windows

When running Cmake, specify "toolchain file for cross-compiling", then pick the file in: vcpkg/scripts/buildsystems/vcpkg.cmake
command line in your build folder e.g MagnumPilotClient/build
    cmake -DCMAKE_TOOLCHAIN_FILE=<path to vcpkg.cmake> ..

then open the generated make/sln file or:
    cmake --build .
    
