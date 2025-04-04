#!/bin/bash

# ---------------------------------------------------------------------------------------------------------
# Bash script for building OpenVDB from sources.
# ---------------------------------------------------------------------------------------------------------

DOWNLOAD=ON

OPENVDB_INSTALL_DIR="$HOME/packages/openvdb"
OPENVDB_DEPENDENCIES_DIR="$HOME/packages/openvdb/openvdb-deps"

BUILDSHARED=ON
BUILDDEBUG=ON

# ------------------------------------------------------------------------
# Allow overriding installation directory through command line argument
if [ -n "$1" ]; then
  OPENVDB_INSTALL_DIR="$1"
fi

if [ "$DOWNLOAD" == "ON" ]; then
  rm -rf download_openvdb
  mkdir download_openvdb

  # Download dependencies
  echo "Cloning TBB from GitHub"
  wget -q -O download_openvdb/tbb.zip https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2020.3.zip
  unzip -q download_openvdb/tbb.zip -d download_openvdb
  git clone https://github.com/oneapi-src/oneTBB.git download_openvdb/oneTBB
  TBB_SOURCE_DIR="download_openvdb/oneTBB"

  echo "Downloading boost 1.80.0 release from SourceForge"
  wget -q -O download_openvdb/boost.zip https://sourceforge.net/projects/boost/files/boost/1.80.0/boost_1_80_0.zip
  unzip -q download_openvdb/boost.zip -d download_openvdb
  BOOST_SOURCE_DIR="download_openvdb/boost_1_80_0"

  echo "Downloading latest zlib release from SourceForge"
  wget -q -O download_openvdb/zlib.zip https://www.zlib.net/zlib131.zip
  unzip -q download_openvdb/zlib.zip -d download_openvdb
  ZLIB_SOURCE_DIR="download_openvdb/zlib-1.3.1"

  echo "Downloading latest blosc release from GitHub"
  wget -q -O download_openvdb/blosc.zip https://github.com/Blosc/c-blosc/archive/refs/tags/v1.21.5.zip
  unzip -q download_openvdb/blosc.zip -d download_openvdb
  BLOSC_SOURCE_DIR="download_openvdb/c-blosc-1.21.5"

  echo "Downloading OpenVDB 11.0.0 release from GitHub"
  wget -q -O download_openvdb/openvdb.zip https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v11.0.0.zip
  unzip -q download_openvdb/openvdb.zip -d download_openvdb
  OPENVDB_SOURCE_DIR="download_openvdb/openvdb-11.0.0"
fi

# ------------------------------------------------------------------------

rm -rf "$OPENVDB_INSTALL_DIR" "$OPENVDB_DEPENDENCIES_DIR"

# --- TBB ----------------------------------------------------------------
cmake -B build_tbb -S "$TBB_SOURCE_DIR" -DCMAKE_INSTALL_PREFIX="$OPENVDB_DEPENDENCIES_DIR/TBB" -DTBB_TEST=OFF
cmake --build build_tbb --config Release
cmake --install build_tbb --config Release
if [ "$BUILDDEBUG" == "ON" ]; then
  cmake --build build_tbb --config Debug
  cmake --install build_tbb --config Debug
else
  echo "Skipping TBB Debug build"
fi

# --- BOOST --------------------------------------------------------------
cd "$BOOST_SOURCE_DIR"
./bootstrap.sh
./b2 variant=release address-model=64 link=static,shared
if [ "$BUILDDEBUG" == "ON" ]; then
  ./b2 variant=debug address-model=64 link=static,shared
else
  echo "Skipping BOOST Debug build"
fi
cd ../../
cp -r "$BOOST_SOURCE_DIR" "$OPENVDB_DEPENDENCIES_DIR/boost_1_80_0"

# --- ZLIB ---------------------------------------------------------------
cmake -B build_zlib -S "$ZLIB_SOURCE_DIR" -DCMAKE_INSTALL_PREFIX="$OPENVDB_DEPENDENCIES_DIR/zlib"
cmake --build build_zlib --config Release
cmake --install build_zlib --config Release
if [ "$BUILDDEBUG" == "ON" ]; then
  cmake --build build_zlib --config Debug
  cmake --install build_zlib --config Debug
else
  echo "Skipping ZLIB Debug build"
fi

# --- BLOSC --------------------------------------------------------------
cmake -B build_blosc -S "$BLOSC_SOURCE_DIR" -DCMAKE_INSTALL_PREFIX="$OPENVDB_DEPENDENCIES_DIR/blosc"
cmake --build build_blosc --config Release
cmake --install build_blosc --config Release
if [ "$BUILDDEBUG" == "ON" ]; then
  cmake --build build_blosc --config Debug
  cmake --install build_blosc --config Debug
else
  echo "Skipping BLOSC Debug build"
fi

# --- OPENVDB ------------------------------------------------------------
cmake -B build_openvdb -S "$OPENVDB_SOURCE_DIR" -DCMAKE_INSTALL_PREFIX="$OPENVDB_INSTALL_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DOPENVDB_BUILD_CORE=ON \
  -DOPENVDB_CORE_SHARED=ON \
  -DOPENVDB_BUILD_NANOVDB=ON \
  -DUSE_NANOVDB=ON \
  -DNANOVDB_USE_OPENVDB=ON \
  -DNANOVDB_USE_CUDA=ON \
  -DNANOVDB_CUDA_KEEP_PTX=ON \
  -DNANOVDB_USE_TBB=ON \
  -DNANOVDB_USE_ZLIB=ON \
  -DUSE_BLOSC=ON \
  -DUSE_TBB=ON \
  -DUSE_ZLIB=ON \
  -DUSE_STATIC_DEPENDENCIES=OFF \
  -DNANOVDB_BUILD_EXAMPLES=ON \
  -DBlosc_INCLUDE_DIR="$OPENVDB_DEPENDENCIES_DIR/blosc/include" \
  -DBlosc_LIBRARY_DEBUG="$OPENVDB_DEPENDENCIES_DIR/blosc/lib/libblosc.so" \
  -DBlosc_LIBRARY_RELEASE="$OPENVDB_DEPENDENCIES_DIR/blosc/lib/libblosc.so" \
  -DBoost_INCLUDE_DIR="$OPENVDB_DEPENDENCIES_DIR/boost_1_80_0/" \
  -DTbb_INCLUDE_DIR="$OPENVDB_DEPENDENCIES_DIR/TBB/include" \
  -DZLIB_INCLUDE_DIR="$OPENVDB_DEPENDENCIES_DIR/zlib/include" \
  -DZLIB_LIBRARY_DEBUG="$OPENVDB_DEPENDENCIES_DIR/zlib/lib/libz.so" \
  -DZLIB_LIBRARY_RELEASE="$OPENVDB_DEPENDENCIES_DIR/zlib/lib/libz.so"

cmake --build build_openvdb --config Release
cmake --install build_openvdb --config Release

echo "Done building OpenVDB"

# --- Move NanoVDB Cuda header files to utils/cuda directory
UTIL_DIR="$OPENVDB_INSTALL_DIR/OpenVDB/include/nanovdb/util"
DEST_DIR="$OPENVDB_INSTALL_DIR/OpenVDB/include/nanovdb/util/cuda"

mkdir -p "$DEST_DIR"
mv "$UTIL_DIR"/*.cuh "$DEST_DIR"
mv "$UTIL_DIR"/GpuTimer.h "$DEST_DIR"

echo "Done moving NanoVDB Cuda header files"

# ---- Delete downloaded sources
rm -rf download_openvdb

# --- Delete build directories
rm -rf build_tbb build_zlib build_blosc build_openvdb


# #!/bin/bash
# # ---------------------------------------------------------------------------------------------------------
# # Bash script for building OpenVDB from sources on Linux.
# # ---------------------------------------------------------------------------------------------------------

# # REQUIRED PACKAGES to install from apt
# # sudo apt install libboost-iostreams-dev libtbb-dev libboost-iostreams-dev libblosc-dev

# DOWNLOAD=ON

# OPENVDB_INSTALL_DIR="$HOME/Packages/openvdb"
# OPENVDB_DEPENDENCIES_DIR="$HOME/Packages/openvdb/openvdb-deps"

# BUILDSHARED=ON
# BUILDDEBUG=OFF

# BUILDSYSTEM="Ninja Multi-Config"

# # Allow overriding installation directory through command line argument
# if [ "$1" != "" ]; then
#    OPENVDB_INSTALL_DIR=$1
# fi

# if [ "$DOWNLOAD" = "ON" ]; then
#     rm -rf download_openvdb 2>/dev/null
#     mkdir -p download_openvdb

#     # Download OpenVDB
#     echo "Downloading OpenVDB 11.0.0 release from GitHub"
#     wget https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v11.0.0.zip -O download_openvdb/openvdb.zip
#     unzip download_openvdb/openvdb.zip -d download_openvdb
#     OPENVDB_SOURCE_DIR="download_openvdb/openvdb-11.0.0"
# fi

# # Clean install directory
# rm -rf ${OPENVDB_INSTALL_DIR} ${OPENVDB_DEPENDENCIES_DIR}
# mkdir -p ${OPENVDB_INSTALL_DIR} ${OPENVDB_DEPENDENCIES_DIR}

# # Build OpenVDB
# rm -rf build_openvdb
# mkdir build_openvdb
# cmake -G "${BUILDSYSTEM}" -B build_openvdb -S ${OPENVDB_SOURCE_DIR} -DCMAKE_INSTALL_PREFIX=${OPENVDB_INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release \
# -DCMAKE_CXX_STANDARD=17 \
# -DOPENVDB_BUILD_CORE=ON \
# -DOPENVDB_CORE_SHARED=ON \
# -DOPENVDB_BUILD_NANOVDB=ON \
# -DUSE_NANOVDB=ON \
# -DNANOVDB_USE_OPENVDB=ON \
# -DNANOVDB_USE_CUDA=ON \
# -DNANOVDB_CUDA_KEEP_PTX=ON \
# -DNANOVDB_USE_TBB=ON \
# -DNANOVDB_USE_ZLIB=ON \
# -DUSE_BLOSC=ON \
# -DUSE_TBB=ON \
# -DUSE_ZLIB=OFF \
# -DUSE_STATIC_DEPENDENCIES=OFF \
# -DNANOVDB_BUILD_EXAMPLES=ON \

# cmake --build build_openvdb --config Release
# cmake --install build_openvdb --config Release

# # Move NanoVDB Cuda header files to utils/cuda directory
# UTIL_DIR="$HOME/Packages/openvdb/OpenVDB/include/nanovdb/util"
# DEST_DIR="$HOME/Packages/openvdb/OpenVDB/include/nanovdb/util/cuda"

# mv ${UTIL_DIR}/*.cuh ${DEST_DIR}
# mv ${UTIL_DIR}/GpuTimer.h ${DEST_DIR}

# # Delete downloaded sources and build directories
# echo "Deleting build directories"
# rm -rf build_tbb
# rm -rf build_zlib
# rm -rf build_blosc
# rm -rf build_openvdb