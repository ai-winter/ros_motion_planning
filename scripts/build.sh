#!/bin/bash

add_path_to_bashrc() {
    local TARGET_PATH="$1"
    local ENV_PATH_NAME="$2"

    if [ -z "$TARGET_PATH" ]; then
        echo "Please provide the target_path."
        return 1
    fi

    if echo "$$ENV_PATH_NAME" | grep -q "$TARGET_PATH"; then
        echo "$TARGET_PATH exists in $$ENV_PATH_NAME."
    else
        if grep -q "export $ENV_PATH_NAME=.*$TARGET_PATH" ~/.bashrc; then
            echo "$TARGET_PATH has been set."
        else
            echo "export $ENV_PATH_NAME=\$$ENV_PATH_NAME:$TARGET_PATH" >> ~/.bashrc
            echo "$TARGET_PATH is set."
        fi
    fi
}

process_3rd_library() {
    local list=("$@")
    for item in "${list[@]}"; do
        sub_dirs=$(find "$HOME/.conan/data/$item/" -mindepth 1 -maxdepth 1 -type d)
        for sub_item in $sub_dirs; do
            sub_packages=$(find "$sub_item/_/_/package" -mindepth 1 -maxdepth 1 -type d)
            for sub_pack in $sub_packages; do
                export LIBRARY_PATH=$LIBRARY_PATH:"$sub_pack/lib"
                export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$sub_pack/lib"
                add_path_to_bashrc "$sub_pack/lib" "LD_LIBRARY_PATH"
                add_path_to_bashrc "$sub_pack/lib" "LIBRARY_PATH"
            done
        done
    done
}

cd ../3rd
conan install . --output-folder=build --build=missing -s compiler.libcxx=libstdc++11
process_3rd_library "gflags" "glog" "ceres-solver" "osqp" "libunwind"
cd ../
catkin_make
