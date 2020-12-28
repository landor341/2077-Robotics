/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

import org.usfirst.frc.team2077.sensors.PotentialSensor;

public class ZPlaceHolderPotentialSensor extends CommandBase {

  public ZPlaceHolderPotentialSensor() {
  }

  @Override
  public void initialize() {
    System.out.println(" ----------------------Something happened---------------------- ");
    robot_.potentialSensor_.setOnNetwork();
    System.out.println("Something 2");
  }

  @Override
  public void execute() {
    System.out.println(" ----------------------iiiiiiiiiiiiiiiiii---------------------- ");
    // PotentialSensor.setOnNetwork();
    // robot_.PotentialSensor.setOnNetwork();
    robot_.potentialSensor_.testing();
    
  }

   @Override
   public void end(boolean interrupted) {
   }

   @Override
   public boolean isFinished() {
     return true;
   }
}
