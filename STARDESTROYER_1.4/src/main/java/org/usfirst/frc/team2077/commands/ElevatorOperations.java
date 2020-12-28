package org.usfirst.frc.team2077.commands;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorOperations extends CommandBase{
///---Mapping---///

// private final VictorSPX smallLift;
// private final VictorSPX largeLift;

    // This class is currently not under development and is a placeholder

    
    public ElevatorOperations(double speed) {

    }
    
    public ElevatorOperations(boolean direction) {
        // largeLift = new VictorSPX(1);
        // largeLift.set(ControlMode.PercentOutput,0.5);//For a test
    }

    public void moveItUpElevator(boolean direction_) {
        VictorSPX smallLift = new VictorSPX(0);
        smallLift.set(ControlMode.PercentOutput,0.5);//For a test
        // smallLift.set(1,0.5);//For a test
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}