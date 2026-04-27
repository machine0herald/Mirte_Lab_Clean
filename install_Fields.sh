mkdir -p build;
cd build;
cmake ..;
make -j$(nproc);

sudo make install;

sudo apt install swig python3-pytest

cmake -DBUILD_PYTHON=ON ..;
make -j$(nproc);
sudo make install;

cd .. && touch /COLCON_IGNORE;