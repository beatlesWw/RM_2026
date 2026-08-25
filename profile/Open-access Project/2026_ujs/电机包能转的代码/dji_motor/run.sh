mkdir build
cd build
rm CMakeCache.txt
rm -rf CMakeFiles
cmake -G Ninja ..
ninja -j24
cd ..