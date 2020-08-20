echo "Building g2o ..."

cd EXTERNAL/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MARCH_NATIVE=OFF
make -j3

cd ../../../