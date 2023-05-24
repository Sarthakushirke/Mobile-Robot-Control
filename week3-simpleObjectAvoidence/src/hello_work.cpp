#include "utilis.h"

int main()
{
	// Create IO object, which will initialize the io layer
	emc::IO io;
	

	printingObject& printingObject = printingObject::getInstance();
	// Create Rate object, which will help keep the loop at a fixed frequency
	emc::Rate r(10);
	ObstacleDetector& detector = ObstacleDetector::getInstance(); // can not be copied

	int ref_theta = 0;
	vector3f dir_vector{0.5, 0, 0};
	
	vector<obstacle> obstacles;
	emc::OdometryData odomData;
	emc::OdometryData odomData_prev;


	// Loop while we are properly connected
	while (io.ok())
	{
		dir_vector.max_angle =120;  //

		// Create an object / variable that will hold the laser data.
		emc::LaserData scan;

		// Send a reference to the base controller (vx, vy, vtheta)
		const float minimum_threshold = 0.7;   // Minimum threshold for obstacle detection


		if (io.readLaserData(scan))
		{
			io.readOdometryData(odomData);
			obstacles.clear();
			// Constants for obstacle detection
			const double forward_angle = M_PI_4;		   // 45 degrees
			const double angle_increment = scan.angle_increment;	

			// Calculate the range of indices to search for obstacles
			const int range_search = static_cast<int>(forward_angle / angle_increment);   //195.6 DEGREES
			const int start_index = (scan.ranges.size() / 2) - range_search;  //500 -195.6 = 303
			const int end_index = (scan.ranges.size() / 2) + range_search;    //500 + 195.6 = 696

			// Variables for obstacle detection
			int begin_idx = -1;
			bool detecting_obstacle = false;    
			bool ShouldMove = true;
			// always try to go forward

			float start_beta_obstacle = start_index;   //303
			// Iterate through the range of indices and detect obstacles
			int beta_treshold = 40;    
			for (int i = start_index; i <= end_index; i++)
			{
				const float cur_range = scan.ranges[i];   // RANGE(303)
				float angle_beta = detector.getBeta(scan.angle_increment,start_index, i+1,scan.ranges);
				// if(abs(angle_beta- prev)<0.2){cout<<"Same"<<endl;}
				// cout<<"Beta: "<<angle_beta<<endl;
				if(cur_range < minimum_threshold && !detecting_obstacle)
				{
					detecting_obstacle = true; 
					begin_idx =i;
					start_beta_obstacle = detector.getBeta(scan.angle_increment,i, i+1,scan.ranges); // start beta obst
				}
				else if (detecting_obstacle==true && abs(start_beta_obstacle-angle_beta)<=beta_treshold)
				{
					continue;    //If entered in this loop it would go to return (come out of the for loop)
				}
				else
				{
					if (detecting_obstacle==true)    // Detecting the wall
					{
						if(debug)printingObject.printCurrent();
						// End of obstacle
						const int end_idx = i - 1;
						const int obstacle_length = end_idx - begin_idx + 1;
						
						const float obstacle_angle_begin = detector.getAngle(angle_increment, begin_idx);
						const float obstacle_angle_end = detector.getAngle(scan.angle_increment, end_idx);
						const float obstable_angle = abs(obstacle_angle_end -obstacle_angle_begin);
						if(debug)printingObject.printCurrent();

						detecting_obstacle = false;
						if (!obstacles.empty() && abs(begin_idx - obstacles.back().end) < 5)
						{
							obstacles.back().end = begin_idx;
							if(debug) printingObject.printCurrent();
						}
						else
						{
						if(debug)printingObject.printCurrent();
						// Create obstacle object
						obstacle new_obstacle(begin_idx, end_idx);

						// Update obstacle angle
						new_obstacle.updateAngles(obstacle_angle_begin,obstacle_angle_end);

						// push
						// cout<<"Pushed! Was : "<<obstacles.size() <<" | update: " << obstacles.size()+1<<endl;
						obstacles.push_back(new_obstacle);		
						if(debug)printingObject.printCurrent();
						}
					}
				}
			}
			float maxAngle = detector.getMAxDegree(scan.angle_increment, obstacles);
			float idxMaxAngle = detector.getIdxAngle(scan.angle_increment,maxAngle);
			
			dir_vector.updateThetaObstacle(obstacles.size()>0);
			// cout<<"DIR Vector x:"<< dir_vector.x<<" y: " << dir_vector.y<<"theta: "<< dir_vector.theta<<endl;
			io.sendBaseReference(dir_vector.x,0, dir_vector.theta); // rotate dir_vector.x
			cout<<scan.angle_increment<<endl;
			// Questoion 1+2
			// printingObject.printingOdomData(odomData);
			// printingObject.printingOdomDataDifference(odomData, odomData_prev, true);
		}
		else
		{
			cout << "Failed to read laser data." << endl;
		}

		// Sleep to maintain the loop rate
		r.sleep();
	}

	return 0;
}
