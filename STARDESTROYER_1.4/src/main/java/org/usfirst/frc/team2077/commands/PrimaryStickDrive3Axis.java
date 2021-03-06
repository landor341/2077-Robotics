/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

public class PrimaryStickDrive3Axis extends CommandBase {

  public PrimaryStickDrive3Axis() {
    addRequirements(robot_.position_);
  }

  @Override
  public void execute() {

    // Speed limit as a percentage (0.0-1.0) of maximum wheel speed
    double speedLimit = 1.0;
    // Rotation limit as a percentage (0.0-1.0) of maximum wheel speed
    double rotationLimit = 1.0; // 0.3;
    // Acceleration/deceleration limit, in Gs
    // Should be at or below the static coefficient of friction (CoF) between the wheels and the floor
    // Too high allows wheelspin, too low is hazardous due to slow deceleration
    //double accelerationLimit = 1.0;
    double accelerationLimit = robot_.constants_.STARDESTROYER_ACCELERATION_G_LIMIT; // TODO: Put acceleration/deceleration limits in constants
    double decelerationLimit = robot_.constants_.STARDESTROYER_DECELERATION_G_LIMIT; // TODO: Put acceleration/deceleration limits in constants

    if (robot_.analogSettings_ != null) {
      double[] dialSetting = { // analog input dials, scaled to 0.0 - 1.0
        robot_.analogSettings_.get(3),
        robot_.analogSettings_.get(2),
        robot_.analogSettings_.get(1)
      };

      double speedLimitMin = 0.2;
      double speedLimitMax = 1.0;
      speedLimit = speedLimitMin + (speedLimitMax-speedLimitMin) * dialSetting[0];
     
      double rotationLimitMin = 0.2;
      double rotationLimitMax = 1.0;
      rotationLimit = rotationLimitMin + (rotationLimitMax-rotationLimitMin) * dialSetting[1];
      
      double accelerationLimitMin = .05;
      double accelerationLimitMax = 0.5;
      accelerationLimit = accelerationLimitMin + (accelerationLimitMax-accelerationLimitMin) * (1 - dialSetting[2]); // reverse dial
      //decelerationLimit = Math.max(accelerationLimit, .25); // don't let this go too low for safety
    }
    
    robot_.chassis_.setGLimits(accelerationLimit, decelerationLimit);

    // TODO: Who handles rotation updates if another command owns robot_position_?
    // TODO: Check joystick/drive capabilities and merge w/2-axis.
    double north = DriveStation.adjustInputSensitivity(-robot_.driveStation_.primaryStick_.getY(), .2, 2.5);
    double east = DriveStation.adjustInputSensitivity(robot_.driveStation_.primaryStick_.getX(), .2, 2.5);
    if (CommandScheduler.getInstance().requiring(robot_.heading_) != null) {
      // heading controlled elsewhere, just do position here
      //System.out.println(" STICK(3): " + north + " \t" + east);
      robot_.chassis_.setVelocity01(north * speedLimit, east * speedLimit);
    }
    else {
      // heading controlled here, do both position and rotation
      double clockwise = DriveStation.adjustInputSensitivity(robot_.driveStation_.primaryStick_.getTwist(), .2, 2.5);
      //System.out.println(" STICK(2): " + north + " \t" + east + " \t" + clockwise);
      robot_.chassis_.setVelocity01(north * speedLimit, east * speedLimit, clockwise * rotationLimit);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
