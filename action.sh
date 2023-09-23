# pip3 install -e .
cd build
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release -DPIP_INSTALL=OFF 

make
