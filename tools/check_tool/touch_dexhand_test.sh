#!/bin/bash

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")
cd $current_script_dir

if [ -e /dev/stark_serial_touch_L ]; then
    if [ -e /etc/udev/rules.d/stark_serial_touch_L.rules ]; then
        echo ""
    else
        echo "/etc/udev/rules.d/stark_serial_touch_L.rules does not exist."
    fi
else
    echo "/dev/stark_serial_touch_L does not exist."
fi

if [ -e /dev/stark_serial_touch_R ]; then
    if [ -e /etc/udev/rules.d/stark_serial_touch_R.rules ]; then
        echo ""
    else
        echo "/etc/udev/rules.d/stark_serial_touch_R.rules does not exist."
    fi
else
    echo "/dev/stark_serial_touch_R does not exist."
fi

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:"$current_script_dir/touch_dexhand/dist/shared/linux"

# 解析参数
test_rounds=5
scan=false
test=false
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --test) test=true;test_rounds="${2:-5}"; shift ;;
        --scan) scan=true ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

if [ "$scan" = true ]; then
    ./touch_dexhand/touch_dexhand_test --scan
elif [ "$test" = true ]; then
    ./touch_dexhand/touch_dexhand_test --test "$test_rounds"
else 
    echo "Usage: $0 [--scan] [--test <test_rounds>]"
fi