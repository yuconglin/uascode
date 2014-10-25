gnome-terminal \
	--tab --title "roscore" --command "bash -c \"
				roscore;
				exec bash\""  \
	--tab --title "mavros" --command "bash -c \"
				env sleep 3s;
				rosrun mavros mavros_node _fcu_url:='tcp://127.0.0.1:5763'
				exec bash\""  \
	--tab --title "planNode3" --command "bash -c \"
		                env sleep 3s;
                                rosrun uascode test_PlanNode3 /home/yucong/yucong_codes_git/sitl/ardupilot/Tools/autotest/ap_large.txt
				exec bash\""  \
