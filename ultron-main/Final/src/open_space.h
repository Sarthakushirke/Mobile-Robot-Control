
#include "../src/json_restaurant.cpp"

#include "./bresanham.h"

#include "../include/objects/Robot.h"

#include "../include/DataTypes.h"

bool doorStatus = true;




void occGridMap(Robot robot,  RobotStates botState)
{

	std::vector<coordinates> indices;

	// std::vector<double> scanArc = robot._scanArc;

	emc::LaserData laserData = robot.getLaserData();

	// sector sect = robot.arcLength;

	double log_occ = 1.0, log_free = 0.5;

	for(int i = 0; i < laserData.ranges.size(); i+=1)
	{
		coordinates mapLaser, mapBot;

		double angle = (double(i) - laserData.ranges.size()/2)*laserData.angle_increment + botState.a - M_PI/2;  //Bot angle from world to map

		angle = wrapToPi(angle);

		mapBot = sim_world_to_map(botState.Pose);

		mapLaser = bot_to_map(laserData.ranges[i], angle, mapBot);

		coordinates wLaser = sim_map_to_world(mapLaser);

		

		//Get laser trajectory in grid coord
		indices = Bresenham( botState.Pose, wLaser, GRID_LENGTH);
		
		//Update freecells until hit point
		int j;
		for(j = 0; j < indices.size(); ++j)
		{
			if(indices[j].x >=0 && indices[j].x < n_col_grid && indices[j].y >=0 && indices[j].y < n_row_grid)
			{
				int x_ind = int(indices[j].x);
				int y_ind = int(indices[j].y);

				oMap[y_ind][x_ind].logOdd -= log_free;

				if(oMap[y_ind][x_ind].gridID != -1)
					gridList[oMap[y_ind][x_ind].gridID].logOdd -= log_free; 

				//Thresholding max 100
				if(oMap[y_ind][x_ind].logOdd < -100)
				{
					oMap[y_ind][x_ind].logOdd = -100;

					if(oMap[y_ind][x_ind].gridID != -1)
						gridList[oMap[y_ind][x_ind].gridID].logOdd = -100;
				}
					
			}
		}
		
		//Occupied cells update fo rhit
		if(laserData.ranges[i] < laserData.range_max && laserData.ranges[i] > laserData.range_min )
		{
				
			//Update occupied
			int x_ind = std::ceil(wLaser.x/GRID_LENGTH);
			int y_ind = std::ceil(wLaser.y/GRID_LENGTH);

			int conn_count = 2;
			if(x_ind >=0 && x_ind < n_col_grid && y_ind >=0 && y_ind < n_row_grid)
			{
				oMap[y_ind][x_ind].logOdd += log_occ;

				int numRows = oMap.size();
				int numCols = oMap[0].size();

				int layers = 3;

				// Define the range of rows and columns to search within
				int startRow = std::max(y_ind - layers, 0);
				int endRow = std::min(y_ind + layers, numRows - 1);
				int startCol = std::max(x_ind - layers, 0);
				int endCol = std::min(x_ind + layers, numCols - 1);

				// Iterate through the defined range
				for (int row = startRow; row <= endRow; ++row) 
				{
					for (int col = startCol; col <= endCol; ++col) 
					{
						oMap[row][col].logOdd += log_occ;

						if(oMap[row][col].gridID != -1)
						{
							gridList[oMap[row][col].gridID].logOdd += log_occ; 

						}

						if(oMap[row][col].logOdd > 100)
						{
							oMap[row][col].logOdd = 100;
							if(oMap[row][col].gridID != -1)
								gridList[oMap[row][col].gridID].logOdd = 100;
						}


					}
				}
			}
				
				
	
			

		}



	}
}



void fill_scan_log(Robot &robot, RobotStates botState)
{
	
	emc::LaserData laserData = robot.getLaserData();


	for(int i = 0; i<robot._openSpace.size(); ++i)
	{
		if( abs(robot._openSpace[i].startID - robot.arcLength.startID) > 3 &&
		    abs(robot._openSpace[i].endID - robot.arcLength.endID) > 3)
		{
			for(int j = robot._openSpace[i].startID; j < robot._openSpace[i].endID; j++)
			{
				coordinates mapLaser, mapBot;

				double angle = (double(j) - laserData.ranges.size()/2)*laserData.angle_increment + botState.a - M_PI/2;

				angle = wrapToPi(angle);

				mapBot = sim_world_to_map(botState.Pose);

				mapLaser = bot_to_map(robot._scanArc[i], angle, mapBot);

				coordinates wLaser = sim_map_to_world(mapLaser);
			
					
				//Update occupied
				int x_ind = std::ceil(wLaser.x/GRID_LENGTH);
				int y_ind = std::ceil(wLaser.y/GRID_LENGTH);

				if(x_ind >=0 && x_ind < n_col_grid && y_ind >=0 && y_ind < n_row_grid)
				{
					oMap[y_ind][x_ind].objFlag = true;

					if(oMap[y_ind][x_ind].gridID != -1)
						gridList[oMap[y_ind][x_ind].gridID].objFlag = true; 

							
				}
			}

		}	



	}
	

	
}

bool open_space(double x_next, double y_next, double goalTol, Robot &robot, int prev_t, int path_node_ID)
{
    coordinates goal;

    emc::Rate r(REFRESH_RATE);    

	emc::LaserData laserData;
    emc::OdometryData odom; 
    

	bool odomUpdate = false, laserUpdate = false;


    while(!odomUpdate || !laserUpdate)
	{
		// r.sleep();
		if(robot.newOdomData())
		{
			odom = robot.getOdomData();
			odomUpdate = true;
		}
		if(robot.newLaserData())
		{
			laserData = robot.getLaserData();
			laserUpdate = true;
		}

	}
		
	goal.x = x_next;

	goal.y = y_next;

    double fov = M_PI;
	
	double angTol = fov/4;
	int counter = 0;
	#if DEBUG_MODE
	std::cout<<"current pose: "<< odom.x<<' '<< odom.y<<std::endl;
	std::cout<<"Enter next Pose: ";
	std::cin>>goal.x>>goal.y;
	
	#endif

	

    if(goal.x<999 && goal.y<999)
	{
		coordinates Pose(odom.x, odom.y);
				

		coordinates headPose, headErr, goalErr;
		double headAng;

		goalErr.x = Pose.x - goal.x;
		goalErr.y = Pose.y - goal.y;

		double goalErr_a = wrapToPi(odom.a - atan2(goal.y - Pose.y, goal.x - Pose.x));

		headAng = wrapToPi(odom.a - atan2(goal.y - Pose.y, goal.x - Pose.x));


		double horizonLimit = 0.7, radius;

		odomUpdate = false;
		laserUpdate  = false;

		emc::LaserData scan;


		int counter =0;


		while(goalErr.x*goalErr.x + goalErr.y*goalErr.y >= goalTol*goalTol )
		{
			
			
			if(robot.newOdomData())
			{
				odom = robot.getOdomData();
				odomUpdate = true;
			}
			if(robot.newLaserData())
			{
				scan = robot.getLaserData();
				laserUpdate = true;
			}

			
			Pose.x = odom.x; 
			Pose.y = odom.y;
			goalErr.x = Pose.x - goal.x;
			goalErr.y = Pose.y - goal.y;

			bool flag = false; //Flag for sector read
			
			sector sect;

			bool talk = false;

			if(gridList[path_node_ID].doorFlag == true && doorStatus && goalErr.x*goalErr.x + goalErr.y*goalErr.y < 0.7*0.7 )
			{

				auto startTime = std::chrono::steady_clock::now();
				while(true)
				{
					robot.sendInputs(0,0,0);
					int countDoor = 0;
					

					if(robot.newLaserData() && robot.newOdomData())
					{
						odom = robot.getOdomData();
						RobotStates botState(odom.x, odom.y, odom.a);

						coordinates Pose(odom.x,odom.y);
						botState.Pose = sim_map_to_world(Pose);

						botState.a = odom.a + M_PI/2;
									

						occGridMap(robot, botState);



						for(int j =0; j< doorID.size(); ++j)
						{
							if(gridList[doorID[j]].logOdd > 5)
								countDoor++;

						}

						if(talk == false && countDoor > 20)
						{    
							doorStatus = true;
							#if SIMULATOR
							robot.open_door_sim();
							#else
							robot.voiceCall("Please open the door");
							#endif
							talk =true;
						}
						if(countDoor < 10)
						{    
							doorStatus = false;
							break;
						}

						auto currentTime  = std::chrono::steady_clock::now();
						auto elapsedTime  = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
						if(elapsedTime > WAIT_TIME)
						{    
							
							doorStatus = false;
							break;
						}
					}

					

				
					
				}



			}

			if(laserUpdate)
			{
			
				int headSectID, sectID;
				double headSectLen;
		
				// shrink radius
				
				if(prev_t !=-1)
				{ 
					if(sqrt(goalErr.x*goalErr.x + goalErr.y*goalErr.y) - horizonLimit < 0.001)
					{
						radius = sqrt(goalErr.x*goalErr.x + goalErr.y*goalErr.y);
					}
					else
						radius = horizonLimit;
				}
				else 
					radius = horizonLimit;
				
			
				// index for fov
				sect.startID = laserData.ranges.size()/2 - fov/laserData.angle_increment/2;
				sect.endID = laserData.ranges.size()/2 + fov/laserData.angle_increment/2;

				robot.arcLength = sect;

				robot.createScanArc(radius);

				std::vector<double> scanArc = robot._scanArc;
						
				if(robot.newLaserData() && robot.newOdomData())
				{
					RobotStates botState(odom.x, odom.y, odom.a);

					botState.Pose = sim_map_to_world(Pose);

					botState.a = odom.a + M_PI/2;
					
					
					occGridMap(robot, botState);
				}
				
				
			
				
				#if DRIVE_MODE

				if(robot.openFound && abs(goalErr_a) < angTol)
				{
					double dist2Goal;
					flag = false;

					dist2Goal = 100000; 

					std::vector<sector> openSpace = robot._openSpace;

					std::vector<coordinates> hitPoints = robot._hitPoints;

					bool headFound = false;
						
					//Clearance ifobject detected
					if(robot.objFound)
					{
						for(int i=0; i<openSpace.size(); ++i)
						{
							//closest to goal on arc
							for(int j=openSpace[i].startID; j< openSpace[i].endID; ++j) 
							{
								if(openSpace[i].startID == sect.startID && openSpace[i].endID != sect.endID)
								{
									if((-(double(j) - openSpace[i].endID)*laserData.angle_increment*radius > BOT_CLEARANCE ))
									{
										headAng = (double(j) - scanArc.size()/2)*laserData.angle_increment;
										headFound = true;

										headPose.x = cos(headAng+odom.a)*radius + Pose.x;
										headPose.y = sin(headAng+odom.a)*radius + Pose.y;

										double g_err_x = goal.x - headPose.x;
										double g_err_y = goal.y - headPose.y;

										if(dist2Goal*dist2Goal  > g_err_x*g_err_x + g_err_y*g_err_y )
										{
											headSectID = j;
											sectID  = i;
											dist2Goal = sqrt(g_err_x*g_err_x + g_err_y*g_err_y );  // find minimum distance to goal 								

										}	
									}
									


								}
								else if(openSpace[i].endID == sect.endID && openSpace[i].startID != sect.startID)
								{
									if((double(j) - openSpace[i].startID)*laserData.angle_increment*radius > BOT_CLEARANCE ) 
									{
										headAng = (double(j) - scanArc.size()/2)*laserData.angle_increment;
										headFound = true;

										headPose.x = cos(headAng+odom.a)*radius + Pose.x;
										headPose.y = sin(headAng+odom.a)*radius + Pose.y;

										double g_err_x = goal.x - headPose.x;
										double g_err_y = goal.y - headPose.y;

										if(dist2Goal*dist2Goal  > g_err_x*g_err_x + g_err_y*g_err_y )
										{
											headSectID = j;
											sectID  = i;
											dist2Goal = sqrt(g_err_x*g_err_x + g_err_y*g_err_y );  // find minimum distance to goal 								

										}	
									}
									
								}
								else
								{
									if(((double(j) - openSpace[i].startID)*laserData.angle_increment*radius > BOT_CLEARANCE ) &&
									  (-(double(j) - openSpace[i].endID)*laserData.angle_increment*radius > BOT_CLEARANCE ) )
									{
										headAng = (double(j) - scanArc.size()/2)*laserData.angle_increment;
										headFound = true;

										headPose.x = cos(headAng+odom.a)*radius + Pose.x;
										headPose.y = sin(headAng+odom.a)*radius + Pose.y;

										double g_err_x = goal.x - headPose.x;
										double g_err_y = goal.y - headPose.y;

										if(dist2Goal*dist2Goal  > g_err_x*g_err_x + g_err_y*g_err_y )
										{
											headSectID = j;
											sectID  = i;
											dist2Goal = sqrt(g_err_x*g_err_x + g_err_y*g_err_y );  // find minimum distance to goal 								

										}	

									}


								}

								// if(abs(headAng - goalErr_a) < angTol && headFound == true)
								// {

								
								// 	headPose.x = cos(headAng+odom.a)*radius + Pose.x;
								// 	headPose.y = sin(headAng+odom.a)*radius + Pose.y;

								// 	double g_err_x = goal.x - headPose.x;
								// 	double g_err_y = goal.y - headPose.y;

								// 	if(dist2Goal*dist2Goal  > g_err_x*g_err_x + g_err_y*g_err_y )
								// 	{
								// 		headSectID = j;
								// 		sectID  = i;
								// 		dist2Goal = sqrt(g_err_x*g_err_x + g_err_y*g_err_y );  // find minimum distance to goal 								

								// 	}	

								// }
								// // else
								// // {
								// // 	headFound = false;
								// // 	continue;
								// // }
								
								
								
								
							}
						}
						if(headFound == false)
						{
							odom = robot.getOdomData();
							RobotStates botState(odom.x, odom.y, odom.a);

							coordinates Pose(odom.x,odom.y);
							botState.Pose = sim_map_to_world(Pose);

							botState.a = odom.a + M_PI/2;
							
							fill_scan_log(robot, botState);

							robot.sendInputs(0,0,0);

							
							assign_gridID();
							create_connections();
							Makegridlist();
							get_table_connections();
							link_node_to_table();


							return false;


						}
					
					}
					else //Free run
					{
						
						for(int i=0; i<openSpace.size(); ++i)
						{
							for(int j=openSpace[i].startID; j< openSpace[i].endID; ++j)
							{
								headAng = (double(j) - scanArc.size()/2)*laserData.angle_increment;

								headPose.x = cos(headAng+odom.a)*radius + Pose.x;
								headPose.y = sin(headAng+odom.a)*radius + Pose.y;

								double g_err_x = goal.x - headPose.x;
								double g_err_y = goal.y - headPose.y;

								if(dist2Goal*dist2Goal  > g_err_x*g_err_x + g_err_y*g_err_y )
								{
									headSectID = j;
									sectID  = i;

									dist2Goal = sqrt(g_err_x*g_err_x + g_err_y*g_err_y );  // find minimum distance to goal 		
					

								}
							}
						}

						
					}

					
					//Find local heading goal
					headAng = (double(headSectID) - scanArc.size()/2)*laserData.angle_increment;
					
					headPose.x = cos(headAng+odom.a)*radius + Pose.x;
					headPose.y = sin(headAng+odom.a)*radius + Pose.y;
					
					headErr.x = Pose.x - headPose.x;
					headErr.y = Pose.y - headPose.y; 	

					#if SIMULATOR
					robot.draw_io({{Pose.x, Pose.y},{headPose.x, headPose.y}},{0,1,0}, 0.02,2);
					#endif 
					
					
					std::pair<double, double> vel;


					vel = velocity_finder(Pose, odom.a, headPose, hitPoints);

					robot.sendInputs(vel.first, 0 , vel.second);

					
				
				}
			
				else if( abs(goalErr_a) >= angTol )
				{
					
					int odomUpdate = false;
					while(abs(goalErr_a) >= angTol)
					{
						if(robot.newOdomData())
						{
							odom = robot.getOdomData();
							odomUpdate = true;
						}
						if(odomUpdate)
						{
							robot.sendInputs(0, 0, -0.3*goalErr_a/abs(goalErr_a));

							goalErr_a = wrapToPi(odom.a - atan2(goal.y - Pose.y, goal.x - Pose.x));
						}
						r.sleep();
					}	
				}
				else
				{
					robot.sendInputs(0,0,0);

					odom = robot.getOdomData();
					RobotStates botState(odom.x, odom.y, odom.a);

					coordinates Pose(odom.x,odom.y);
					botState.Pose = sim_map_to_world(Pose);

					botState.a = odom.a + M_PI/2;
					
					fill_scan_log(robot, botState);

					robot.sendInputs(0,0,0);

					
					
					assign_gridID();
					create_connections();
					Makegridlist();
					get_table_connections();
					link_node_to_table();
					
					return false;
				}
				#endif

			}
			
			r.sleep();
		}
		
		
		
	}

	return true;
}