rd ".\build" -Recurse -Force -Confirm:$false  
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build -j