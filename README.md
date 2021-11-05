Jeston AGX Xavier 5.11.2021 ".bashrc" file



export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH

alias fanla="sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'"
alias fanlarmısın='sudo /usr/sbin/nvpmodel -d cool'
alias gucle30='sudo nvpmodel -m 4'
alias guclemax='sudo nvpmodel -m 1'
source /opt/ros/melodic/setup.bash


alias cm='catkin_make && source devel/setup.bash'
alias sour='source devel/setup.bash'
alias indir='sudo apt-get install'
#export ROS_MASTER_URI=http://192.168.1.5:11311
#export ROS_IP=192.168.1.5
#export ROS_HOSTNAME=192.168.1.5
alias dulbirakan='export ROS_IP=192.168.1.74'
alias karaoglanla='export ROS_IP=192.168.1.53'
alias silamk='clear'
alias gosterle='sudo nvpmodel -q'


#export DISPLAY=localhost:0.0
source ~/rover_21_workspace/devel/setup.bash
#export DISPLAY=:0.0

