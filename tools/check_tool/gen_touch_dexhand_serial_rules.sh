#!/bin/bash
# list ttyUSB* devices
echo -e "\033[32m\n列出 'ttyUSB*' 设备:\033[0m"
ls -lh /dev/ttyUSB*

read -p "请输入想要绑定的串口号(数字, 例如: 0, 1, 2, 3, ...):" dev
echo "你选择的串口为: /dev/ttyUSB$dev"

if [ ! -e "/dev/ttyUSB$dev" ]; then
    echo -e "\033[31m\nError: /dev/ttyUSB$dev 不存在.\033[0m"
    exit 1
fi

# 询问绑定到左手或右手
while true; do
    read -p "你想绑定到左手还是右手? (l/r): " hand
    case $hand in
        l|L)
            rule_name="stark_serial_touch_L"
            break
            ;;
        r|R)
            rule_name="stark_serial_touch_R"
            break
            ;;
        *)
            echo "无效的输入，请输入 l 或 r."
            ;;
    esac
done

echo "/dev/ttyUSB$dev 被自定义为: $rule_name"

serial=$(udevadm info --attribute-walk --name=/dev/ttyUSB$dev|grep ATTRS{serial} | cut -d= -f3 | sed 's/"//g'|head -n 1)
# # 如果属性不为空，创建一个udev规则文件
if [ -n "$serial" ]; then
echo '正在为序列号为'$serial'的设备生成udev规则...'
echo 'KERNEL=="ttyUSB*", ATTRS{serial}=="'$serial'", MODE:="0777", SYMLINK+="'$rule_name'"' > /etc/udev/rules.d/$rule_name.rules
echo '生成成功! 请重启计算机或者插拔设备以使规则生效。'
else 
echo '未找到序列号，请检查设备是否已连接。'
fi
# 重新加载udev规则
udevadm control --reload-rules
udevadm trigger