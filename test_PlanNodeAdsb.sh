gnome-terminal \
	--tab --title "roscore" --command "bash -c \"
				roscore;
				exec bash\""  \
	--tab --title "planNode2" --command "bash -c \"
				env sleep 3s;
				rosrun uascode test_PlanNode2 /home/yucong/yucong_codes_git/sitl/ardupilot/Tools/autotest/ap_large.txt
				exec bash\""  \
	--tab --title "mavlink_rec" --command "bash -c \"
				env sleep 3s;
				rosrun uascode test_MavRecNode
				exec bash\""  \
	--tab --title "mavlink_send" --command "bash -c \"
				env sleep 3s;
				rosrun uascode test_MavSendNode 1
				exec bash\""  \
	--tab --title "AdsbFromFile" --command "bash -c \"
				env sleep 3s;
			        rosrun uascode test_adsbfile LoadSendConfig.txt 1
				exec bash\""  \
