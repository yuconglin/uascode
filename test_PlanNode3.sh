gnome-terminal \
	--tab --title "roscore" --command "bash -c \"
				roscore;
				exec bash\""  \
	--tab --title "mavros" --command "bash -c \"
				env sleep 3s;
				#rosrun mavros mavros_node _fcu_url:='tcp://127.0.0.1:5763'
				rosrun mavros mavros_node _fcu_url:='udp://127.0.0.1:14550@127.0.0.1:19550'
				exec bash\""  \
        #	--tab --title "mavwp" --command "bash -c \"
        #                        rosrun mavros mavwp
        #				exec bash\""  \
	--tab --title "planNode3" --command "bash -c \"
		                env sleep 3s;
                                rosrun uascode test_PlanNode3 /home/yucong/yucong_codes_git/sitl/ardupilot/Tools/autotest/ap_large.txt
				exec bash\""  \
        --tab --title "obsFromFile" --command "bash -c \"
				env sleep 3s;
			        rosrun uascode test_obsfile /records/obss_log.txt /data/20140924-212359obs.txt LoadSendConfigSimu.txt 2 
				exec bash\""  \
