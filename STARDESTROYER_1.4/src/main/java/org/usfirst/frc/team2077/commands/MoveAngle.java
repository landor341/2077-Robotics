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

public class MoveAngle extends CommandBase {
  private double finishTime;
  private double dps_;
  private double angle_;
  private double distance;
  private double finalHeading;


  // as is degrees/second
  
  public MoveAngle(double angle) {
    this(angle, 5);
    System.out.println("**************" + angle + "****************");
  }

  public MoveAngle(double angle, double dps) {
    angle_ = angle;
    dps_ = dps;
    addRequirements(robot_.heading_);
  }

  @Override
  public void initialize() {

    finalHeading = robot_.chassis_.getPosition()[2] + angle_;
    //distance = robot_.constants_.TESTBOT_TRACK_WIDTH;
  }

  @Override
  public void execute() {
    //System.out.println("###############################################");
    if (angle_ < 0) {
      if (robot_.chassis_.getPosition()[2] > finalHeading) {
        robot_.chassis_.setRotation(- dps_);  
      }
    } else {
      if (robot_.chassis_.getPosition()[2] < finalHeading) {
        robot_.chassis_.setRotation(dps_);
      } 
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (angle_ > 0) {
      if (robot_.chassis_.getPosition()[2] >= finalHeading) {
        return true;
      }
    } else { //rotation is negative
      if (robot_.chassis_.getPosition()[2] <= finalHeading) {
        return true;
      }
    }
    return false;
  }
}
