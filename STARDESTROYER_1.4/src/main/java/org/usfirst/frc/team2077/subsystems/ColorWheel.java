/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.subsystems;

import static org.usfirst.frc.team2077.Robot.*;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase {

    public Talon talon_ = new Talon(9);
}
