gnome-terminal \
	--tab --title "roscore" --command "bash -c \"
                                roscore;
				exec bash\"" \
	--tab --title "mavros" --command "bash -c \"
				env sleep 3s;
                                #rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0  _gcs_url:='udp://@localhost' _mission/pull_after_gcs:='true'
				roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:57600"  gcs_url:="udp://:14555@10.206.164.162:14550" mission/pull_after_gcs:="true"
				exec bash\"" \
	--tab --title "MavrosListen" --command "bash -c \"
				env sleep 3s;
				#rosrun uascode test_MavrosListen wp_r home_alt
				rosrun uascode test_MavrosListen 25 690.15
				exec bash\"" \
