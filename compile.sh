mkdir -p ./build
cd ./build
cmake ..
make -j $(nproc --ignore=2)
cd ..
#sudo ./install.sh
