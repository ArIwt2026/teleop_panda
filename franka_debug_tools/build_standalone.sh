#!/bin/bash

# Configuration
SNAP7_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../external/snap7"
SNAP7_INCLUDE="$SNAP7_ROOT/release/wrappers/c-cpp"
SNAP7_SOURCE="$SNAP7_ROOT/release/wrappers/c-cpp/snap7.cpp"
SNAP7_LIB="$SNAP7_ROOT/build/bin/linux/libsnap7.so"

function compile_bridge() {
    local src_file=$1
    local out_file=$2
    
    echo "Compiling $src_file..."
    g++ -O3 \
        "$src_file" \
        $SNAP7_SOURCE \
        -I$SNAP7_INCLUDE \
        $SNAP7_LIB \
        -Wl,-rpath,"$SNAP7_ROOT/build/bin/linux" \
        -lpthread -lrt \
        -o "$out_file"

    if [ $? -eq 0 ]; then
        echo ">> Success! Executable created: ./$out_file"
    else
        echo ">> Error: Compilation of $src_file failed."
        exit 1
    fi
}

compile_bridge "plc_bridge.cpp" "plc_bridge_standalone"
compile_bridge "hochregal_bridge.cpp" "hochregal_bridge_standalone"

echo ""
echo ">> All builds complete."
echo ">> To run original: ./plc_bridge_standalone"
echo ">> To run hochregal: ./hochregal_bridge_standalone"
