#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    currentOdometry = pose_xyt_t();
    deltaOdometryAction = pose_xyt_t();
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
        // code checks to see if the robot has moved
    // if so, it computes the new action as the delta X,Y,Z,Theta of this action
    // and the previous one
    // if not, we return false and update the current odometry (simply to ensure time is updated)
    // the false return val is an optimization to prevent certain downstream code from running
    if(odometry.x == currentOdometry.x 
        && odometry.y == currentOdometry.y 
        && odometry.theta == currentOdometry.theta
        && odometry.utime >= currentOdometry.utime)
    {
        currentOdometry = odometry;
        return false;
    }
    else
    {
        // if we've moved, create an action as the delta of this and the previous odometry
        deltaOdometryAction.x = odometry.x - currentOdometry.x;
        deltaOdometryAction.y = odometry.y - currentOdometry.y;
        deltaOdometryAction.theta = odometry.theta - currentOdometry.theta;
        deltaOdometryAction.utime = odometry.utime;
        currentOdometry = odometry;
        return true;
    }

}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    // given a passed in particle, computes a new particle which is the application of our action
    // to the provided one.
    // TODO: change the action to be a sample from a distribution as opposed to a naive application
    // also, do velocity based movement (eg distance @ heading) as opposed to just x/y/theta application
    double distanceTraveled = sqrt(pow(deltaOdometryAction.x,2) + pow(deltaOdometryAction.y,2));
    double deltaTheta = deltaOdometryAction.theta;
    double deltaX = deltaOdometryAction.x;
    double deltaY = deltaOdometryAction.y;
    double alpha = atan2(deltaY, deltaX) - sample.pose.theta;
    std::random_device rd;
    // Mersenne twister PRNG, initialized with seed random device instance
    std::mt19937 gen(rd());
    std::normal_distribution<double> turnOneDist(0.0,0.01*std::abs(alpha));
    std::normal_distribution<double> travelDist(0.0,0.05*distanceTraveled); 
    std::normal_distribution<double> turnTwoDist(0.0,0.01*std::abs(deltaTheta - alpha));
    particle_t toReturn = particle_t(sample);
    toReturn.parent_pose = sample.pose;
    double e1 = turnOneDist(gen);
    double e2 = travelDist(gen);
    double e3 = turnTwoDist(gen);
    toReturn.pose.x =  sample.pose.x + (distanceTraveled+e2)*cos(sample.pose.theta+alpha+e1);
    toReturn.pose.y =  sample.pose.y + (distanceTraveled+e2)*sin(sample.pose.theta+alpha+e1);;
    toReturn.pose.theta = sample.pose.theta + (deltaTheta + e1 + e3);
    toReturn.pose.utime = sample.pose.utime;
    toReturn.parent_pose = sample.pose;
    return toReturn;
}
