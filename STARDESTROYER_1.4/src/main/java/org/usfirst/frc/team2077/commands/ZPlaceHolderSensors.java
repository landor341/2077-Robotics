/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

import org.usfirst.frc.team2077.sensors.PotentialSensor;

public class ZPlaceHolderSensors extends CommandBase {

  public ZPlaceHolderSensors() {
  }

  @Override
  public void initialize() {
    // robot_.potentialSensor_.setOnNetwork();
    // System.out.println("Something 2");
  }

  @Override
  public void execute() {
    //robot_.infraredSensor_.getStatusIsLoaded();
  }

   @Override
   public void end(boolean interrupted) {
   }

   @Override
   public boolean isFinished() {
     return true;
   }
}
