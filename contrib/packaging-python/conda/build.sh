
echo Started build.sh
mkdir ./build
cd ./build
export NP_INCL=$(python3 -c "import numpy; print(numpy.get_include())")
echo $NP_INCL

# Python libraries are different file types for MacOS and linux
# TODO: Check if this is needed since MacOS has its own deployment script

if [ `uname` == Darwin ]; then
    PY_LIB="libpython${PY_VER}.dylib"
else
    PY_LIB="libpython${PY_VER}.so"
fi

ROS_SETUP_SCRIPT="/opt/ros/humble/setup.bash"
if [ -f "$ROS_SETUP_SCRIPT" ]; then
  source $ROS_SETUP_SCRIPT
fi

# set MKL vars
export MKL_INTERFACE_LAYER=LP64
export MKL_THREADING_LAYER=INTEL

CONFIGURATION=Release
# Configure step
cmake -G "Ninja" -DCMAKE_INSTALL_PREFIX=$PREFIX \
 -DCMAKE_PREFIX_PATH=$PREFIX \
 -DCMAKE_SYSTEM_PREFIX_PATH=$PREFIX \
 -DCH_CONDA_INSTALL=ON \
 -DCH_INSTALL_PYTHON_PACKAGE=$SP_DIR \
 -DCH_PYCHRONO_DATA_PATH=../../../../share/chrono/data \
 -DCH_PYCHRONO_SHADER_PATH=../../../../lib/sensor_ptx \
 -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3.10 \
 -DPYTHON_INCLUDE_DIR:PATH=/usr/include/python3.10 \
 -DPYTHON_LIBRARY:FILEPATH=/usr/lib/x86_64-linux-gnu/libpython3.10.so \
 -DCMAKE_BUILD_TYPE=$CONFIGURATION \
 -DENABLE_MODULE_IRRLICHT=ON \
 -DENABLE_MODULE_POSTPROCESS=ON \
 -DENABLE_MODULE_VEHICLE=ON \
 -DENABLE_MODULE_PYTHON=ON \
 -DENABLE_MODULE_SENSOR=ON \
 -DENABLE_MODULE_ROS=ON \
 -DENABLE_MODULE_PARSERS=ON \
 -DUSE_CUDA_NVRTC=OFF \
 -DCUDA_ARCH_NAME=Manual \
 -DCUDA_ARCH_PTX=52 \
 -DCUDA_ARCH_BIN=5.2 \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 -DENABLE_MODULE_CASCADE=OFF \
 -DOpenCASCADE_DIR=$HOME/Packages/opencascade-7.4.0/adm \
 -DENABLE_MODULE_PARDISO_MKL=ON \
 -DMKL_INCLUDE_DIR=$BUILD_PREFIX/include \
 -DMKL_RT_LIBRARY=$BUILD_PREFIX/lib/libmkl_rt.so \
 -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
 -DIRRLICHT_INSTALL_DIR=/usr/include/irrlicht \
 -DOptiX_INSTALL_DIR=/home/harry/optix \
 -DNUMPY_INCLUDE_DIR=/usr/lib/python3/dist-packages/numpy/core/include \
 -Durdfdom_DIR=$HOME/Packages/urdf/lib/urdfdom/cmake \
 -Durdfdom_headers_DIR=$HOME/Packages/urdf/lib/urdfdom_headers/cmake \
 -Dconsole_bridge_DIR=$HOME/Packages/urdf/lib/console_bridge/cmake \
 -Dtinyxml2_DIR=$HOME/Packages/urdf/CMake \
 ./..

#  -DCASCADE_INCLUDE_DIR=$HOME/miniconda3/include/opencascade \
#  -DCASCADE_LIBDIR=$HOME/miniconda3/lib \

# Build & Install
ninja
ninja install
