gnome-terminal \
        --tab --title "roscore" --command "bash -c \"
				roscore;
				exec bash\""  \
	--tab --title "PredictNode" --command "bash -c \"
				env sleep 3s;
				rosrun uascode test_LoopPredict /home/yucong/yucong_codes_git/sitl/ardupilot/Tools/autotest/ap1.txt
				exec bash\""  \
	--tab --title "mavlink_rec" --command "bash -c \"
				env sleep 3s;
				rosrun uascode test_MavRecNode
				exec bash\""  \
	--tab --title "mavlink_send" --command "bash -c \"
				env sleep 3s;
				rosrun uascode test_MavSendNode
				exec bash\""  \
	--tab --title "obsFromFile" --command "bash -c \"
				env sleep 3s;
				rosrun uascode test_obsfile
				exec bash\""  \
