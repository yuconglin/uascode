gnome-terminal \
	--tab --title "roscore" --command "bash -c \"
				roscore; 
				exec bash\""  \
	--tab --title "receiver" --command "bash -c \"
	                        env sleep 3s;
				rosrun uascode test_PlanNode
	                        exec bash\""  \
	--tab --title "sender" --command "bash -c \"
	                        env sleep 3s ;
	                        rosrun uascode test_obsfile
	                        exec bash\""  \
