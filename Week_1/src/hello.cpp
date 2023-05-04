#include <emc/io.h>
#include <emc/rate.h>
#include <iostream> // Add this line under the other includes
#include <algorithm>
int main()
{
 // Create IO object, which will initialize the io layer
 emc::IO io;
 // Create Rate object, which will help keeping the loop at a fixed frequency
 emc::Rate r(10);
 // Loop while we are properly connected
 while(io.ok())
 {
  // create an object / variable that will hold the laser data.
 emc::LaserData scan;
 // Send a reference to the base controller (vx, vy, vtheta)

float tresh=0.2;
float min_dis = 1000; 
 if (io.readLaserData(scan))
{
	 // Loop through all the laser range measurements
	for(int i =0; i<scan.ranges.size(); ++i)
	{
		float cur_range = scan.ranges[i];

		if(cur_range< min_dis) min_dis = cur_range;
	}
	std::cout<<"min:" <<min_dis<<std::endl;
	if(min_dis< tresh){  io.sendBaseReference(0, 0, 0);}
	else{ 
	std::cout<<"run:"<<std::endl;
	 io.sendBaseReference(0.5, 0, 0);}

}
else
{
   std::cout << "Failed to read laser data" << std::endl;
}
// Sleep remaining time
 r.sleep();
}
 return 0;
}
