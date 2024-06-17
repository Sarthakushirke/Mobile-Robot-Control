
#include "../src/json_restaurant.cpp"

#include "./bresanham.h"

#include "../include/objects/Robot.h"

#include "../include/DataTypes.h"



double botClearance = 0.20;




void occGridMap(Robot robot,  RobotStates botState)
{

	std::vector<coordinates> indices;

	std::vector<double> scanArc = robot._scanArc;

	emc::LaserData laserData = robot.getLaserData();

	sector sect = robot.arcLength;

	double log_occ = 1.0, log_free = 0.7;

	for(int i = sect.startID; i < sect.endID; ++i)
	{
		coordinates hitLaser;

		double angle = (double(i) - scanArc.size()/2)*laserData.angle_increment;

		hitLaser = laser_to_world(scanArc[i], angle, botState);

		//Get laser trajectory in grid coord
		indices = Bresenham(botState.Pose, hitLaser, gridLength);
		
		//Update freecells until hit point
		for(int j = 0; j < indices.size() - 1; ++j)
		{
			if(indices[j].x >=0 && indices[j].x < n_col_grid && indices[j].y >=0 && indices[j].y < n_row_grid)
			{
				int x_ind = int(indices[j].x);
				int y_ind = int(indices[j].y);

				oMap[y_ind][x_ind].logOdd -= log_free;
				//Thresholding max 100
				if(oMap[y_ind][x_ind].logOdd < -100)
					oMap[y_ind][x_ind].logOdd = -100;

			}
		}
	
		if(scanArc[i] < robot._radius)
		{
				
			//Update occupied
			int x_ind = int(indices.back().x);
			int y_ind = int(indices.back().y);
			if(x_ind >=0 && x_ind < n_col_grid && y_ind >=0 && y_ind < n_row_grid)
			{
				oMap[x_ind][y_ind].logOdd += log_occ;
				if(oMap[y_ind][x_ind].logOdd > 100)
						oMap[y_ind][x_ind].logOdd = 100;
			}

		}
		else
		{
			//Update Free
			int x_ind = int(indices.back().x);
			int y_ind = int(indices.back().y);
			if(x_ind >=0 && x_ind < n_col_grid && y_ind >=0 && y_ind < n_row_grid)
			{
				oMap[x_ind][y_ind].logOdd -= log_occ;
				//thresholding min -100
				if(oMap[y_ind][x_ind].logOdd < -100)
						oMap[y_ind][x_ind].logOdd = -100;
			}
		}


	}
}



void open_space(double x_next, double y_next, double goalTol)
{
    coordinates goal;

    emc::Rate r(20);
    emc::IO io;
	emc::IO emc_io;

    Robot robot(&io);

	emc::LaserData laserData;
    emc::OdometryData odom; 

	bool odomUpdate = false, laserUpdate = false;
	
	while(!odomUpdate || !laserUpdate)
	{

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


	double angTol = 0.1;

	goal.x = x_next;

	goal.y = y_next;

    double fov = M_PI;
	
	#if DEBUG_MODE
	std::cout<<"current pose: "<< odom.x<<' '<< odom.y<<std::endl;
	std::cout<<"Enter next Pose: ";
	std::cin>>goal.x>>goal.y;
	#endif

    if(goal.x<999 && goal.y<999)
	{
		coordinates Pose(odom.x, odom.y);
		RobotStates botState(Pose, odom.a);
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

			
			//std::cout<<"got odom data"<<std::endl;
			Pose.x = odom.x; 
			Pose.y = odom.y;
			goalErr.x = Pose.x - goal.x;
			goalErr.y = Pose.y - goal.y;

			bool flag = false; //Flag for sector read
			
			sector sect;

			if(laserUpdate)
			{
			

				int headSectID, sectID;
				double headSectLen;
		

				// shrink radius
				
				if(sqrt(goalErr.x*goalErr.x + goalErr.y*goalErr.y) - horizonLimit < 0.1)
				{
					radius = sqrt(goalErr.x*goalErr.x + goalErr.y*goalErr.y);
				}
				else
					radius = horizonLimit;

								
				// index for fov
				sect.startID = laserData.ranges.size()/2 - fov/laserData.angle_increment/2;
				sect.endID = laserData.ranges.size()/2 + fov/laserData.angle_increment/2;

				robot.arcLength = sect;

				robot.createScanArc(radius);

				std::vector<double> scanArc = robot._scanArc;
						
				
				// occGridMap(robot, botState);
				
			
				if(robot.openFound)
				{
					double dist2Goal;
					flag = false;

					dist2Goal = 100000; 

					std::vector<sector> openSpace = robot._openSpace;

					std::vector<coordinates> hitPoints = robot._hitPoints;
						
					//Clearance ifobject detected
					if(robot.objFound)
					{
						for(int i=0; i<openSpace.size(); ++i)
						{
							//closest to goal on arc
							for(int j=openSpace[i].startID; j< openSpace[i].endID; ++j)
							{
								if(((double(j) - openSpace[i].startID)*laserData.angle_increment*radius > botClearance ) &&
								(-(double(j) - openSpace[i].endID)*laserData.angle_increment*radius > botClearance ) )
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


					io.sendPath({{Pose.x, Pose.y},{headPose.x, headPose.y}},{0,1,0}, 0.02,2);

					
					
					
					std::pair<double, double> vel;


					vel = velocity_finder(Pose, odom.a, headPose, hitPoints);

					io.sendBaseReference(vel.first, 0 , vel.second);

					

				}
			
				else if( abs(goalErr_a) >= angTol )
				{
					while(abs(goalErr_a) >= angTol)
					{
						io.readOdometryData(odom);
						io.sendBaseReference(0,0,-0.6*goalErr_a/abs(goalErr_a));

						goalErr_a = wrapToPi(odom.a - atan2(goal.y - Pose.y, goal.x - Pose.x));
						r.sleep();
					}	
				}
				else
				{
					std::cout<<"Path blocked. Please reroute ;-; "<<std::endl;
				}

			}
			
			r.sleep();
		}
		
	}
}