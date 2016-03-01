/*
 *  Camera.h
 *
 *  Created on: 06.02.16
 *      Author: Stephan Manthe
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace camSurv
{
struct Camera
{
    int cameraId = -1;
    
    Eigen::Vector4d q = Eigen::Vector4d::UnitX();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    
    Eigen::Quaterniond quat() const
	{
    	return Eigen::Quaterniond(q(0),q(1),q(2),q(3));
	}
    void setQuat(const Eigen::Quaterniond& quat)
    {
	    q << quat.w(), quat.x(), quat.y(), quat.z();
	}	
};
}

#endif 
