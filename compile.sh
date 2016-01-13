mkdir -p ./build
cd ./build
cmake ..
make -j $(nproc --ignore=1)
cd ..
#sudo ./install.sh
