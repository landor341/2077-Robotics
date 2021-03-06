/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

public class DoAlmostNothing extends CommandBase {

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
  

  public DoAlmostNothing() {
    System.out.println("========================== DoAlmostNothing()");
  }

  @Override
  public void initialize() {
    System.out.println("========================== initialize()");
    (new DoNothing()).schedule();
  }

  @Override
  public void execute() {
    System.out.println("========================== execute()");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("========================== end(" + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
