
#include "config.h"

class obstacle;  // forward declaration

struct obstacle
{
	int begin, end;
	float angle_begin, angle_end;
	int id;
 	static int total;  				// track how many obstacle we have
	float beta; 

	obstacle() = default;
	obstacle(int begin_, int end_){begin = begin_; end = end_;total++; id = total;}
	obstacle(int begin_, int end_, float angle_begin_, float angle_end_){
		begin = begin_; end = end_;angle_begin = angle_begin_;angle_end = angle_end_;total++;id = total;
	}

	void updateAngle(const float angle_, int binary_cond)  // if b =0 -> begin else end
	{
		if(binary_cond ==0)
			{angle_begin = angle_;} 
		else
			{angle_end = angle_;}
	} 
	void updateAngles(const float angle_begin_, const float angle_end_)  // if b =0 -> begin else end
	{
		angle_begin =angle_begin_;
		angle_end = angle_end_;
	} 
	float get_angle(float increment, int i)const
	{
		return get_angle(increment,i);
	}
	
	int middle(){return (begin+end)/2;}
};


// struct vector3f
struct vector3f {
public:
	float x, y, theta; // theta is either 0.1 or -0.1 -> convert to radiance: - > 180/M_PI*rad
	float speed = 0.5;
	float Odom = 0;
	float totalAngle;
	bool right = false;
	float max_angle;

	vector3f() = default;
	vector3f(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}
	vector3f(float x_, float y_, float theta_, float totalAngle_) : x(x_), y(y_), theta(theta_), totalAngle(totalAngle_) {}

	// operators
	vector3f operator*(const vector3f& vec2) {
		return vector3f(vec2.x * x, vec2.y * y, theta * vec2.theta);
	}

	vector3f operator+(const vector3f& vec2) {
		return vector3f(vec2.x + x, vec2.y + y, theta + vec2.theta);
	}

	std::tuple<float, float> get_X_theta(bool obstacle_detect_) {
		if (obstacle_detect_) {
			updateTheta();
			return std::make_tuple(0, theta);
		}
		else {
			return std::make_tuple(x, 0);
		}
	}

	// functions
	float get_theta() {
		return theta;
	}

	float get_dir(const float ref) {
		if (ref > theta) {
			return -1;
		}
		else {
			return 1;
		}
	}

	float returnMod(const float ref) {
		return std::abs(std::fmod(ref, theta));
	}

	void checkOdomMax() {
		if (std::abs(Odom) > 2) {
			right = !right; // Toggle the value of right using the negation operator
		}
	}

	void checkOdom() {
		checkOdomMax();
		Odom += (right ? 0.1 : -0.1);
	}

	// float thetaInRad(){return }
	void updateTheta() {
		checkOdom();
		theta = (right ? -0.2 : 0.2);
		if(debug) std::cout << "Odom: " << Odom << std::endl;
	}


	void updateThetaObstacle(bool found) {
		if (found) {
			updateTheta();
			theta = 0.0;
		} else {
			theta = 0.2;
		}
		x = (found ? 0.0 : 0.5);
	}
};




// SingleTom
class ObstacleDetector {
public:
    static ObstacleDetector& getInstance() {
        static ObstacleDetector instance;
        return instance;
    }

    ObstacleDetector(const ObstacleDetector&) = delete; // Disable copy constructor
    ObstacleDetector& operator=(const ObstacleDetector&) = delete; // Disable assignment operator

    inline float getAngle(float increment, int i) {
        return increment * i;
    }
	inline int getIdxAngle(float increment, float angle)
	{
		return (int)(angle/increment);
	}

    inline float getAngleMiddle(float increment, int i, int j) {
        return ((j - i) * increment);
    }

	inline int getOpeningSize(obstacle & obs1, obstacle & obs2 )
	{
		return obs2.begin - obs1.end;
	}
    float getBeta(float angleIncrement, int beginIdx, int endIdx, const std::vector<float>& ranges) {
        const float obstacleAngleBegin = getAngle(angleIncrement, beginIdx);
        const float obstacleAngleEnd = getAngle(angleIncrement, endIdx);
        const float obstacleAngle = std::abs(obstacleAngleEnd - obstacleAngleBegin);

        const float distanceBC = std::sqrt(
            ranges[beginIdx] * ranges[beginIdx] +
            ranges[endIdx] * ranges[endIdx] -
            2 * ranges[endIdx] * ranges[beginIdx] * std::cos(obstacleAngle)
        );
        const double beta = std::acos(
            (distanceBC * distanceBC + ranges[beginIdx] * ranges[beginIdx] - ranges[endIdx] * ranges[endIdx]) /
            (2 * ranges[beginIdx] * distanceBC)
        );

        return static_cast<float>(beta); // return in degrees
	}
	
	float getMAxDegree(const float increment, std::vector<obstacle>& detectedObstables) 
	{
		int amontObstacle = detectedObstables.size();	
		float maxDegree;
		float maxOpeningSize = std::numeric_limits<float>::infinity();
		for(int i = 0; i<amontObstacle-1; i++)
		{
			int currrentSize = getOpeningSize(detectedObstables[i], detectedObstables[i+1]); // ended at size-1  so we wont go after the boundries.
			if(currrentSize<maxOpeningSize)
			{
				maxOpeningSize= currrentSize;
				maxDegree  = getAngle(increment,(detectedObstables[i+1].begin + detectedObstables[i].end)/2 ); //*get the middle part*// 
			}
		} // have the max size and the degree
		if(debugObstacleDetector==true) cout<<"Max Degree size: "<<maxOpeningSize<<endl;
		// we should adapt the steering so that go toward the max degree. 
		return maxDegree;
		

	}	
private:
    ObstacleDetector() = default; // Private constructor to prevent instantiation
};

// init obstacle static var to 0
int obstacle::total = 0;


// Singeltom printing
class printingObject
{
public:
	static int currentPrintingVal;
	static printingObject& getInstance()
	{
		static printingObject instance;
		return instance;
	}

	void printingOdomData(emc::OdometryData odomData_)
	{
		cout<<"OdomData----x: " << odomData_.x<< ". y:" <<odomData_.y << ". z: "  <<odomData_.a <<endl; 
	}
	void printingOdomDataDifference(emc::OdometryData & odomData_current, emc::OdometryData & odomData_previous , bool transferOdom =false)
	{
		cout<<"Difference Odom----x: " << odomData_current.x -odomData_previous.x << ". y:" <<odomData_current.y- odomData_previous.y << ". theta: "  <<odomData_current.a- odomData_previous.a <<endl;
		if(transferOdom==true){
		odomData_previous.x = odomData_current.x;
		odomData_previous.y = odomData_current.y;
		odomData_previous.a =odomData_current.a;
		}
	}
	void printCurrent()
	{
		cout<<currentPrintingVal<<endl;
		currentPrintingVal++;
	}
	
private:
	printingObject() = default;
};
int printingObject::currentPrintingVal= 0;