/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.usfirst.frc.team2077.Robot.*;

import java.sql.Time;
import java.util.Timer;

import org.usfirst.frc.team2077.DriveStation;

public class MoveAngleTarget extends CommandBase {
  private double finishTime;
  private double dps_;
  private double angle_;
  private double distance;
  private double finalHeading;
  private boolean moreThan;

  // as is degrees/second


  public MoveAngleTarget() {
    //System.out.println("OOGA BOOGA **************************");
    
    
    addRequirements(robot_.heading_);
  }

  @Override
  public void initialize() {
    //System.out.println("OOGA BOOGA **************************");
    //angle_ = robot_.chassis_.getPosition()[2] % 360 + Math.toDegrees( Math.atan2(robot_.crosshairs_.getTarget()[1] - robot_.chassis_.getPosition()[1], robot_.crosshairs_.getTarget()[0] - robot_.chassis_.getPosition()[0]));
    angle_ = robot_.angleSensor_.getAngle() + robot_.crosshairs_.getAzimuth();
    finalHeading = angle_;
    moreThan = false;
    //if (angle_ > robot_.chassis_.getPosition()[2]) {
      if (angle_ > robot_.angleSensor_.getAngle()) {
      moreThan = true;
    }
    dps_ = 35;
    //distance = robot_.constants_.TESTBOT_TRACK_WIDTH;
  }

  @Override
  public void execute() {
    //System.out.println("###############################################");
    //System.out.println(robot_.angleSensor_.getAngle() + "      " + angle_ + moreThan);
    if (!moreThan) {
      //if (robot_.chassis_.getPosition()[2] > angle_) {
      if (robot_.angleSensor_.getAngle() > angle_) {
        //System.out.println("DONE 1 DONE 1 DONE 1 DONE 1 DONE 1 DONE 1 "); 
        robot_.chassis_.setRotation(-dps_);  
      }
    } else {
      
      //if (robot_.chassis_.getPosition()[2] < angle_) {
      if (robot_.angleSensor_.getAngle() < angle_) {
        //System.out.println("DONE 2 DONE 2 DONE 2 DONE 2 DONE 2 DONE 2 ");
        robot_.chassis_.setRotation(dps_);
      } 
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (moreThan) {
      //if (robot_.chassis_.getPosition()[2] >= finalHeading) {
      if (robot_.angleSensor_.getAngle() >= finalHeading) {
        return true;
      }
    } else { //rotation is negative
      if (robot_.angleSensor_.getAngle() <= finalHeading) {
      //if (robot_.chassis_.getPosition()[2] <= finalHeading) {
        return true;
      }
    }
    return false;
  }
}
