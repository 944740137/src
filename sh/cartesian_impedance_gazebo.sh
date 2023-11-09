if [ $# -gt 0 ]; then
    # 使用提供的参数
    my_parameter=$1
else
    # 默认参数
    my_parameter="false"
fi

echo "是否带夹抓：$my_parameter"

source devel/setup.bash
roslaunch franka_gazebo panda.launch x:=-0.5 \
    controller:=cartesian_impedance_controller \
    use_gripper:=$my_parameter \
    rviz:=true    
