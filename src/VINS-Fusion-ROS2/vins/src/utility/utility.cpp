// /*******************************************************
//  * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
//  * 
//  * This file is part of VINS.
//  * 
//  * Licensed under the GNU General Public License v3.0;
//  * you may not use this file except in compliance with the License.
//  *******************************************************/

// #include "utility.h"

// Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
// {
//     Eigen::Matrix3d R0;
//     Eigen::Vector3d ng1 = g.normalized();
//     Eigen::Vector3d ng2{0, 0, 1.0};
//     R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
//     double yaw = Utility::R2ypr(R0).x();
//     R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
//     // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
//     return R0;
// }

 #include "utility.h"

 Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
 {
     // Avoid unnecessary normalization if g is already unit length
     const double g_norm = g.norm();
     const Eigen::Vector3d ng1 = (std::abs(g_norm - 1.0) < 1e-6) ? g : (g / g_norm);
     
     // Use a static constant for the reference vector
     static const Eigen::Vector3d ng2(0, 0, 1.0);
     
     // Compute rotation directly using the shortest arc method instead of 
     // using the more expensive Quaternion::FromTwoVectors
     const double dot = ng1.dot(ng2);
     
     // Check if vectors are nearly parallel or anti-parallel
     if (dot > 0.99999) {
         // Vectors are nearly identical - return identity
         return Eigen::Matrix3d::Identity();
     } else if (dot < -0.99999) {
         // Vectors are nearly opposite - need to find an axis
         // Use an axis perpendicular to ng1 to rotate around
         Eigen::Vector3d axis = Eigen::Vector3d(1, 0, 0).cross(ng1);
         if (axis.norm() < 1e-6) {
             // If ng1 is aligned with x-axis, use y-axis instead
             axis = Eigen::Vector3d(0, 1, 0).cross(ng1);
         }
         axis.normalize();
         
         // 180 degree rotation around axis
         Eigen::AngleAxisd rotation(M_PI, axis);
         Eigen::Matrix3d R0 = rotation.toRotationMatrix();
         
         double yaw = Utility::R2ypr(R0).x();
         return Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
     } else {
         // Normal case - compute rotation directly
         const Eigen::Vector3d axis = ng1.cross(ng2).normalized();
         const double angle = std::acos(dot);
         
         Eigen::AngleAxisd rotation(angle, axis);
         Eigen::Matrix3d R0 = rotation.toRotationMatrix();
         
         // Extract yaw and compensate
         double yaw = Utility::R2ypr(R0).x();
         return Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
     }
 }
