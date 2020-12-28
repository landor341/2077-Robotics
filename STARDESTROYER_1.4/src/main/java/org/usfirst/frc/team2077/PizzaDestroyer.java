/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import org.usfirst.frc.team2077.commands.AimCrosshairs;
import org.usfirst.frc.team2077.commands.Move2;
import org.usfirst.frc.team2077.commands.Nudge;
import org.usfirst.frc.team2077.commands.PrimaryStickDrive3Axis;
import org.usfirst.frc.team2077.commands.RangeToCrosshairs;
import org.usfirst.frc.team2077.commands.Rotate;
import org.usfirst.frc.team2077.commands.SecondaryStickDrive;
import org.usfirst.frc.team2077.commands.SteerToCrosshairs;
import org.usfirst.frc.team2077.commands.TrackTarget;
import org.usfirst.frc.team2077.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.MecanumChassis;
import org.usfirst.frc.team2077.drivetrain.PizzabotDriveModule;
import org.usfirst.frc.team2077.sensors.AnalogSettings;
import org.usfirst.frc.team2077.sensors.AngleSensor;
import org.usfirst.frc.team2077.subsystems.Crosshairs;
import org.usfirst.frc.team2077.subsystems.DummyLauncher;
import org.usfirst.frc.team2077.subsystems.Telemetry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// The default Command template from WPI splits a lot of top level declaration
// and initialization into a separate RobotContainer class. The main purpose of
// doing so seems to be to make the code less convenient to read :)
// In this example such code has been merged back into this file. - RAB

// The following header comment from the template file is just plain wrong.
// It applies to frc.robot.Main. Probably a cut/paste error as code evolved. - RAB
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class PizzaDestroyer extends Robot {

  /**
   * Overrides Robot.robotInit().
   */
  @Override
  public void robotInit() {

    // Container for remote control software objects.
    driveStation_ = new DriveStation();

    // Communications with other processes on the DS.
    networkTableInstance_ = NetworkTableInstance.getDefault();

    // Sensors.
    angleSensor_ = new AngleSensor();

    analogSettings_ = new AnalogSettings(1, 2, 3); // NavX MPX Analog In 1 2 3

    //potentialSensor_ = new PotentialSensor();

    // Drivetrain.
    DriveModuleIF[] backwardPizzaDriveModules = { // Pizzabot with front/rear swapped
      new PizzabotDriveModule(3, 18, 19, false),  // southwest (left rear)
      new PizzabotDriveModule(4, 20, 21, false),   // northwest (left front)
      new PizzabotDriveModule(1, 10, 11, true),  // northeast (right front)
      new PizzabotDriveModule(2, 12, 13, true)  // southeast (right rear)
    };
    /*
    DriveModule[] dummyMecanumDriveModules = {
      new DummyDriveModule(100),  // northeast (right front)
      new DummyDriveModule(100),  // southeast (right rear)
      new DummyDriveModule(100),  // southwest (left rear)
      new DummyDriveModule(100)   // northwest (left front)
    };
    */

    chassis_ = new MecanumChassis(backwardPizzaDriveModules, constants_.PIZZABOT_WHEELBASE, constants_.PIZZABOT_TRACK_WIDTH, constants_.PIZZABOT_WHEEL_RADIUS);
    
    // Subsystems.
    //   These dummy subsystems support sepearable command ownership of robot motion and rotation.
    position_ = new SubsystemBase() {};
    heading_ = new SubsystemBase() {};
    target_ = new SubsystemBase() {};

    telemetry_ = new Telemetry();
        
    //   Ball launcher.
    launcher_ = new DummyLauncher(12 /*height*/);
    //(new ResetTarPos()).initialize();
    //tgrabber_ = new TestGrabber();

    //   Manages an operator-controlled aiming point displayed on DS and used for targeting.     
    crosshairs_ = new Crosshairs("Camera0", constants_.FISHEYE_CAMERA_PIXEL_WIDTH, constants_.FISHEYE_CAMERA_PIXEL_HEIGHT, constants_.FISHEYE_CAMERA_FOCAL_LENGTH,
      12 /*camera n*/, 8 /*camera height*/, constants_.FISHEYE_CAMERA_TILT, constants_.UPPER_TARGET_HEIGHT); 

     // Initial robot and target position.
    robot_.chassis_.setPosition(-168, 0, 0); // TODO: Initialize from Smart Dashboard
    double[] p = robot_.chassis_.getPosition();
    robot_.crosshairs_.set(Math.atan2(-p[1], -p[0]), Math.sqrt(p[0]*p[0] + p[1]*p[1]));
    
    System.out.println("CROSSHAIRS:" + crosshairs_);

    // Default teleop commands.
    //drive_ = new PrimaryStickDrive2Axis();
    drive_ = new PrimaryStickDrive3Axis();
    aim_ = new AimCrosshairs();
    track_ = new TrackTarget();
    range_ = new RangeToCrosshairs();

    // Subsystem default commands.
    CommandScheduler.getInstance().setDefaultCommand(position_, drive_);
    // Uncomment the following to move rotation to secondary stick.
    //CommandScheduler.getInstance().setDefaultCommand(heading_, new SecondaryStickDrive());
    CommandScheduler.getInstance().setDefaultCommand(target_, track_);
    CommandScheduler.getInstance().setDefaultCommand(crosshairs_, aim_);
    CommandScheduler.getInstance().setDefaultCommand(launcher_, range_);


    //driveStation_.secondaryTrigger_.whileActiveContinuous(new SteerToCrosshairs());
    //driveStation_.secondaryTrigger_.whileHeld(new ContinousAimToTarget3());
    //driveStation_.secondaryTrigger_.whileHeld(new SteerToCrosshairs());
    driveStation_.secondaryTrigger_.whileActiveContinuous(new SteerToCrosshairs());

    // TODO: Conflicts w/ above, move to another button.
    //driveStation_.secondaryTrigger_.whenPressed(new ToggleLauncher(-((robot_.driveStation_.secondaryStick_.getThrottle() + 1.)/ 2.) * robot_.tlauncher_.launchMaxRPM));


    //driveStation_.secondary5_.whileHeld(new LoadLauncher());  // TODO: debug
    //driveStation_.secondary6_.whileHeld(new AngleLauncher());
    //driveStation_.primaryTrigger_.whileHeld(new RunGrabber());


    // Test code.
    /*
    (new JoystickButton(driveStation_.primaryStick_, 3)).whenPressed(new Move2(40, 0, -180));
    (new JoystickButton(driveStation_.primaryStick_, 4)).whenPressed(new Move2(40, 0, 180));
    (new JoystickButton(driveStation_.primaryStick_, 5)).whenPressed(new Move2(20, 0, -90));
    (new JoystickButton(driveStation_.primaryStick_, 6)).whenPressed(new Move2(20, 0, 90));

    (new POVButton(driveStation_.primaryStick_, 0)).whenPressed(new Nudge(0, .1));
    (new POVButton(driveStation_.primaryStick_, 90)).whenPressed(new Nudge(90, .15));
    (new POVButton(driveStation_.primaryStick_, 180)).whenPressed(new Nudge(180, .1));
    (new POVButton(driveStation_.primaryStick_, 270)).whenPressed(new Nudge(270, .15));

    (new JoystickButton(driveStation_.primaryStick_, 7)).whenPressed(new Move2(12, -12));
    (new JoystickButton(driveStation_.primaryStick_, 8)).whenPressed(new Move2(12, 12));
    (new JoystickButton(driveStation_.primaryStick_, 9)).whenPressed(new Move2(270));
    (new JoystickButton(driveStation_.primaryStick_, 10)).whenPressed(new Move2(-270));
    (new JoystickButton(driveStation_.primaryStick_, 11)).whenPressed(new SequentialCommandGroup(
      new Move2(60, 0, 270),
      //new Move2(25.5, 0),
      new Move2(60, 0, 270),
      new Move2(60, 0, -270),
      //new Move2(25.5, 0),
      new Move2(60, 0, -270)));
    (new JoystickButton(driveStation_.primaryStick_, 12)).whenPressed(new SequentialCommandGroup(
      new Move2(20, 0),
      new Move2(-90),
      new Move2(20, 0),
      new Move2(-90),
      new Move2(40, 0),
      new Move2(-90),
      new Move2(40, 0),
      new Move2(-90),
      new Move2(40, 0),
      new Move2(-90),
      new Move2(20, 0),
      new Move2(-90),
      new Move2(20, 0),
      new Move2(-180)));

    */
    (new JoystickButton(driveStation_.secondaryStick_, 6)).whenPressed(new Move2(40, 0));
    (new JoystickButton(driveStation_.secondaryStick_, 7)).whenPressed(new Move2(0, 40));
    (new JoystickButton(driveStation_.secondaryStick_, 8)).whenPressed(new Move2(90));
    (new JoystickButton(driveStation_.secondaryStick_, 9)).whenPressed(new Move2(180));
    (new JoystickButton(driveStation_.secondaryStick_, 10)).whenPressed(new Move2(20, 0));
    (new JoystickButton(driveStation_.secondaryStick_, 11)).whenPressed(new Move2(0, 20));




    //(new JoystickButton(driveStation_.secondaryStick_, 6)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getVoltageTest());
    //(new JoystickButton(driveStation_.secondaryStick_, 7)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getRatioTest());
    //(new JoystickButton(driveStation_.secondaryStick_, 8)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getRevolutionsTest(10, .2));
    //(new JoystickButton(driveStation_.secondaryStick_, 6)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getVelocityTest(32, .5));
    //(new JoystickButton(driveStation_.secondaryStick_, 7)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getVelocityTest(16, 1));
    //(new JoystickButton(driveStation_.secondaryStick_, 8)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getVelocityTest(8, 2));
    //(new JoystickButton(driveStation_.secondaryStick_, 9)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getVelocityTest(4, 3));
    //(new JoystickButton(driveStation_.secondaryStick_, 10)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getVelocityTest(2, 4));
    //(new JoystickButton(driveStation_.secondaryStick_, 11)).whenPressed(((PizzabotDriveModule)backwardPizzaDriveModules[0]).getVelocityTest(1, 8));




  }
}
