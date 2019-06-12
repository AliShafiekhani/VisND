//----------------------------------------------//
//           N-D Visualization GUI              //
//                                              //
//        Copyright (c) ViGIR-Lab 2019          //
//          Written by Ali Shfiekhani           //
//----------------------------------------------//

#ifndef INSTANCE_CLOUD_H
#define INSTANCE_CLOUD_H

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


using namespace std;

template<class PointType>
class InstanceCloud{
    typedef typename pcl::PointCloud<PointType>::Ptr PointCloudPtr;

    public:
        InstanceCloud(PointCloudPtr cloud, vector<double> cloud_temperature, unsigned int frameID):
        cloudPtr_(cloud), cloudTempVec_(cloud_temperature), frameID_(frameID){
            kdtree_.setInputCloud(cloud);
        }
        InstanceCloud(PointCloudPtr cloud, cv::Mat cloud_temperature, Eigen::Matrix4f tf, unsigned int frameID):
        cloudPtr_(cloud), cloudTempCV_(cloud_temperature), tf_(tf), frameID_(frameID){
            kdtree_.setInputCloud(cloud);
        }
        PointCloudPtr getCloudPtr(){return cloudPtr_;}
        vector<int> searchPointByRadius(PointType searchPoint, float radius){
            vector<int> pointIdxRadiusSearch;
            vector<float> pointRadiusSquaredDistance;
            kdtree_.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            return pointIdxRadiusSearch;
        }
        vector<double> getCloudTempVec(){return cloudTempVec_;}
        cv::Mat getCloudTempCV(){return cloudTempCV_.clone();}
        unsigned int getCloudFrameID(){return frameID_;}
        Eigen::Matrix4f getCloudTF(){return tf_;}


    private:
        PointCloudPtr          cloudPtr_;
        pcl::KdTreeFLANN<PointType> kdtree_;
        vector<double>         cloudTempVec_;
        cv::Mat                cloudTempCV_;
        Eigen::Matrix4f        tf_;                 // wHpcl
        unsigned int           frameID_;
};

#endif
