alias ros1init="source /opt/ros/noetic/setup.bash && source ~/.bashrc"
alias ros2init="source /opt/ros/foxy/setup.bash && source ~/.bashrc"
alias ros2hinit="source ~/ros2_humble/install/setup.bash"

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
# export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General><Internal><MinimumSocketReceiveBufferSize>11MB</MinimumSocketReceiveBufferSize></Internal></Domain></CycloneDDS>'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export ros_ws="~/autoronto_ws"
export ros_ws_crv="~/catkin_ws"
export ros_ws_test="~/test_catkin_ws"
export ros_ws_integ="~/integ_catkin_ws"
export ros_ws_dev="~/dev_catkin_ws"
export ros_ws_old="~/old_catkin_ws"
export ros_ws_extra="~/extra_catkin_ws"
alias rosdeps="cd $ros_ws && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y"
alias bd="cd $ros_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=YES -DBUILD_TESTING=ON && . install/setup.bash && cp build/compile_commands.json ."
alias bdc="cd $ros_ws_crv && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=YES -DBUILD_TESTING=ON && . install/setup.bash && cp build/compile_commands.json ."
alias bdd="cd $ros_ws_dev && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=YES -DBUILD_TESTING=ON && . install/setup.bash && cp build/compile_commands.json ."
alias bdt="cd $ros_ws_test && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=YES -DBUILD_TESTING=ON && . install/setup.bash && cp build/compile_commands.json ."
alias bdo="cd $ros_ws_old && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=YES -DBUILD_TESTING=ON && . install/setup.bash && cp build/compile_commands.json ."
alias ros2config="source ~/autoronto_ws/install/setup.bash"
alias ros2configc="source ~/catkin_ws/install/setup.bash"
alias ros2configt="source ~/test_catkin_ws/install/setup.bash"
alias ros2configi="source ~/integ_catkin_ws/install/setup.bash"
alias ros2configd="source ~/dev_catkin_ws/install/setup.bash"
alias ros2configo="source ~/old_catkin_ws/install/setup.bash"

alias gozv3o="cd ~/old_catkin_ws/src/ZeusV3"

# alias goart="cd ~/catkin_ws/src/artemis"
alias goart="cd ~/autoronto_ws/src/artemis"
alias goartt="cd ~/test_catkin_ws/src/artemis"
alias goarti="cd ~/integ_catkin_ws/src/artemis"
alias goartd="cd ~/dev_catkin_ws/src/artemis"
alias goarte="cd ~/extra_catkin_ws/src/artemis"
alias goartc="cd ~/catkin_ws/src/artemis"

export MAKEFLAGS="-j 16"

# cuda
export PATH=/usr/local/cuda-11.8/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# ros2hinit
export PATH="/home/autoronto/dev/downloads/tracecompass/trace-compass:$PATH"
export PATH="/usr/local/MATLAB/R2022b/bin:$PATH"

# alias ros2lint='find . -regextype posix-extended -regex "^.*.(hpp|cpp)" | uncrustify -c ~/autoronto/ament_code_style.cfg -F - --replace'
alias ros2lint='ament_uncrustify --reformat .'

export PATH=/usr/local/cuda-11.7/bin:$PATH

ros2hinit
