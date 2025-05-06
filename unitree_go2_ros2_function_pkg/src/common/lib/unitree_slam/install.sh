WorkDir=$(cd $(dirname $0); pwd)
read -p "Please select the type of robot:  GO2(1)  or  B2(2) ? " input
echo "WrokDir=$WorkDir"
Arch=$(uname -m)
echo "CPU Arch=$Arch"
set -e
mkdir -p /usr/local/lib/cmake
mkdir -p /usr/local/lib/$Arch

cp -r $WorkDir/unitree_robotics/include/* /usr/local/include
cp -r $WorkDir/unitree_robotics/lib/$Arch/* /usr/local/lib
cp -r $WorkDir/unitree_robotics/lib/$Arch/* /usr/local/lib/$Arch
cp -r $WorkDir/unitree_robotics/lib/cmake/* /usr/local/lib/cmake

if [ "$input" = "1" ]; then
chmod -R 777 /unitree/module/graph_pid_ws/*
chmod -R 777 /unitree/module/graph_pid_ws
chmod -R 777 /unitree/lib/*
fi

echo "success"
