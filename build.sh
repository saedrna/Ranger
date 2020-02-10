mkdir build
cd build

cmake -GNinja \
      -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
      -DCMAKE_SYSROOT=$CONDA_PREFIX/$HOST/sysroot \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DMosek_INCLUDE_DIR=/home/han/mosek/9.1/tools/platform/linux64x86/h \
      -DMosek_LIBRARY_RELEASE=/home/han/mosek/9.1/tools/platform/linux64x86/bin/libmosek64.so \
      -DMosek_LIBRARY_DEBUG=/home/han/mosek/9.1/tools/platform/linux64x86/bin/libmosek64.so \
      ../

cd ..
