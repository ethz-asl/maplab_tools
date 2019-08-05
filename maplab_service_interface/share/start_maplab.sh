source /home/cerberus/maplab_ws/devel/setup.zsh
roscore &
sleep 3s
rosrun maplab_console maplab_console --v=1 --alsologtostderr --vis_scale 5 -num_hardware_threads 16
