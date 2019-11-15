#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    this.currentOdometry = new pose_xyz_t();
    this.deltaOdometryAction = new pose_xyz_t();
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    // code checks to see if the robot has moved
    // if so, it computes the new action as the delta X,Y,Z,Theta of this action
    // and the previous one
    // if not, we return false and update the current odometry (simply to ensure time is updated)
    // the false return val is an optimization to prevent certain downstream code from running
    if(odometry.x == this.currentOdometry.x 
        && odometry.y == this.currentOdometry.y 
        && odometry.z == this.currentOdometry.z 
        && odometry.theta == this.currentOdometry.theta
        && odometry.utime >= this.currentOdometry.utime)
    {
        this.currentOdometry = odometry;
        return false;
    }
    else
    {
        // if we've moved, create an action as the delta of this and the previous odometry
        this.deltaOdometryAction.x = odometry.x - this.currentOdometry.x;
        this.deltaOdometryAction.y = odometry.y - this.currentOdometry.y;
        this.deltaOdometryAction.z = odometry.z - this.currentOdometry.z;
        this.deltaOdometryAction.theta = odometry.theta - this.currentOdometry.theta;
        this.deltaOdometryAction.utime = odometry.utime;
        this.currentOdometry = odometry;
        return true;
    }
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    // given a passed in particle, computes a new particle which is the application of our action
    // to the provided one.
    // TODO: change the action to be a sample from a distribution as opposed to a naive application
    auto toReturn = new particle_t(sample);
    toReturn.x =  this.deltaOdometryAction.x + sample.x;
    toReturn.y =  this.deltaOdometryAction.y + sample.x;
    toReturn.z =  this.deltaOdometryAction.z + sample.x;
    toReturn.theta =  this.deltaOdometryAction.theta + sample.x;
    toReturn.utime = sample.utime;
    return toReturn;
}
