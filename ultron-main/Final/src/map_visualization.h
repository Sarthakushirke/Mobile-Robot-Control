//
// Created by bob on 6-5-19.
// Modified by Dharshan on 18-06-23
//

#ifndef MY_PROJECT_MAP_VISUALIZATION_H
#define MY_PROJECT_MAP_VISUALIZATION_H

#include <string.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <fstream>
#include <cassert>
#include <iostream>
#include "./json.hpp"
#include "geometric_primitives.h"
#include <string>
#include "DataTypes.h"


class visualization {

public:
    visualization(std::string title_="visualization", double xmin=-1, double xmax=5, double ymin=-1, double ymax=5){
        title = title_;
        pix_per_m = 120;
        x_range.min = xmin;
        x_range.max = xmax;
        y_range.min = ymin;
        y_range.max = ymax;
        image = cv::Mat::zeros( (y_range.max - y_range.min) * pix_per_m, (x_range.max - x_range.min) * pix_per_m, CV_8UC3 );
        textimage = cv::Mat::zeros( (y_range.max - y_range.min) * pix_per_m, (x_range.max - x_range.min) * pix_per_m, CV_8UC3 );
        cv::namedWindow(title_, cv::WINDOW_AUTOSIZE );
    }

    void clear(){
        image = cv::Mat::zeros( (y_range.max - y_range.min) * pix_per_m, (x_range.max - x_range.min) * pix_per_m, CV_8UC3 );
    }

    void set_points(std::vector<Point> points_ ){
        points = points_;
        int index = 0;
        for( auto point : points){
            cv::circle(image, cv::Point(pix_per_m* (point.x-x_range.min) ,pix_per_m*(point.y-y_range.min)),1,cv::Scalar(0,50,255),2,8 );
            //cv::putText(textimage,std::to_string(index),cv::Point(pix_per_m* (point.x-x_range.min) ,pix_per_m*(y_range.max-y_range.min) - pix_per_m*(point.y-y_range.min)), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0),1);
            index++;
        }
    }

    void plot_lines(std::vector<std::pair<Node,Node>> lines_, cv::Scalar color)
    {

        for( int i =0; i<lines_.size(); ++i)
        {
            std::vector<cv::Point> p;
            //std::cout << line.p1 << std::endl;
            cv::Point point1(std::floor(pix_per_m*(lines_[i].first.x-x_range.min)),std::floor(pix_per_m*(lines_[i].first.y-y_range.min)));
            cv::Point point2(std::floor(pix_per_m*(lines_[i].second.x-x_range.min)),std::floor(pix_per_m*(lines_[i].second.y-y_range.min)));
            
            p.push_back(point1);
            p.push_back(point2);
            cv::polylines(image, p,false,color,2);
        }
    }

    void show_always(){
        cv::Mat image_corrected;
        cv::flip(image,image_corrected,0);
        

        cv::imshow( title, image_corrected + textimage);
        cv::waitKey(0);
    }

private:
    double pix_per_m;
    struct Offset_m {double x; double y;} offset_m;
    struct Range {double min; double max;};
    Range x_range;
    Range y_range;
    cv::Mat image;
    cv::Mat textimage;
    std::string title;
    std::vector<Point> points;
};

#endif //MY_PROJECT_MAP_VISUALIZATION_H
