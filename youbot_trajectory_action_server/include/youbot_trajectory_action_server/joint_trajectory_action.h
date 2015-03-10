/******************************************************************************
 * Copyright (c) 2011
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov
 * 
 * modified: S.Isler, 2015
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of GPS GmbH nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/


#ifndef JOINTTRAJECTORYACTION_H
#define	JOINTTRAJECTORYACTION_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

namespace KDL
{
class Trajectory_Composite;
}

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class JointStateObserver;

class JointTrajectoryAction
{
public:
    JointTrajectoryAction(JointStateObserver* jointStateObserver);
    JointTrajectoryAction(JointStateObserver* youbot,
                          double velocityGain, double positionGain, double frequency);
    JointTrajectoryAction(const JointTrajectoryAction& orig);
    virtual ~JointTrajectoryAction();

    void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as);
    
    void jointStateCallback(const sensor_msgs::JointState& joint_state);
    void jointStateCallback(const brics_actuator::JointPositions& position,
                            const brics_actuator::JointVelocities& velocity);

    /// currently unused!
    void setVelocityGain(double velocityGain);
    /// currently unused!
    double getVelocityGain() const;

    
    /// currently unused!
    void setPositionGain(double positionGain);
    /// currently unused!
    double getPositionGain() const;

    void setFrequency(double frequency);
    double getFrequency() const;

    // tells the JointTrajectoryAction class to use position commands when following a trajectory: velocity and/or accelerations in the trajectory are not considered
    void usePositionCommands();
    
    // tells the JointTrajectoryAction class to use velocity commands when following a trajectory: currently the velocities are being recalculated using the KDL library and the positions in the trajectory while discarding velocities and/or accelerations, you might want to change that and do interpolations using the velocities in the trajectory if you want to use the class with velocity commands (Velocity and torque commands weren't necessary for the task for which the class has been revised since timing was of no interest, thus this wasn't done) -> check the getAllCurrentVelocities(...) function
    void useVelocityCommands();
    
    // sets ignore_time_ to false
    void dontIgnoreTime();
    // sets ignore_time to true
    void ignoreTime();


private:
  
    bool use_position_commands_; /// if true then only the positions of the trajectory are used, otherwise the velocities will be used - default is true
    bool ignore_time_; /// if true then each point in the trajectory will be commanded with no regard to the the time in the trajectory. Currently defaults to true because accurate trajectory following was desired and timing was of no interest for the task at hand for which this class was revised
    bool no_interpolation_; /// don't interpolate positions of received trajectory, only resend what is received
    
    double velocityGain; /// used as a weighting factor for the issued command - now unused
    double positionGain; /// used as a weighting factor for the issued command - now unused
    double frequency;

    sensor_msgs::JointState current_state; /// loads the current state of the robot, was used for error calculations -> unnecessary if this is delegated to the PID controller (as it should)
    JointStateObserver* jointStateObserver;

private:
    
    /** returns the velocity given in the trajectory trajectoryComposite at time elapsedTimeInSec. If the elapsed time exceeds the time inside the trajectory, velocity zero is returned
     */
    double getVelocityAtTime( const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec);
    
    /** returns the position given in the trajectory trajectoryComposite at time elapsedTimeInSec
     * @throws std::runtime_error If the elapsed time exceeds the time available in the trajectory
     */
    double getPositionAtTime( const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec);
    
    /** gets all velocities for the current time (using ros::Time::now() ) as given in the trajectory
     * @param startTime the time at which trajectory execution started (only used if ignore_time_ is false)
     * @param currentTime the time for which the velocites are wanted (only used if ignore_time_ is true)
     * @return writes into the vector velocities
     */
    void getAllCurrentVelocities( const KDL::Trajectory_Composite* trajectory,
                     int numberOfJoints,
                     ros::Time startTime,
		     double currentTime,
                     std::vector<double>& velocities);
    
    /** gets all positions for the current time (using ros::Time::now() ) as given in the trajectory
     * @param startTime the time at which trajectory execution started (only used if ignore_time_ is false)
     * @param currentTime the time for which the velocites are wanted (only used if ignore_time_ is true)
     * @return writes into the vector positions
     */
    void getAllCurrentPositions( const KDL::Trajectory_Composite* trajectory,
                     int numberOfJoints,
                     ros::Time startTime,
		     double currentTime,
                     std::vector<double>& positions);
    
    /// returns some weighted velocity error - not used anymore
    double calculateVelocity(double actualAngle,
                             double actualVelocity,
                             const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec);
    
    /// calculates the weighted velocity error for all joints at the current time (ros::Time::now()) - not used anymore
    void controlLoop(const std::vector<double>& actualJointAngles,
                     const std::vector<double>& actualJointVelocities,
                     const KDL::Trajectory_Composite* trajectoryComposite,
                     int numberOfJoints,
                     ros::Time startTime,
                     std::vector<double>& velocities);

    void setTargetTrajectory(double angle1,
                             double angle2,
                             double duration,
                             KDL::Trajectory_Composite& trajectoryComposite);

};

#endif	/* JOINTTRAJECTORYACTION_H */

