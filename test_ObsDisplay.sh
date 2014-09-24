gnome-terminal \
  	--tab --title "roscore" --command "bash -c \"
				roscore;			
				exec bash\"" \
        --tab --title "mavlink_send" --command "bash -c \"
	                        env sleep 3s;
	                        rosrun uascode test_MavSendNode 1
	                        exec bash\""  \
        --tab --title "obsFromFile" --command "bash -c \"
	                        env sleep 3s;
	                        rosrun uascode test_obsfile /records/obss_log.txt /data/20140516-162158obs.txt
	                        exec bash\""  \