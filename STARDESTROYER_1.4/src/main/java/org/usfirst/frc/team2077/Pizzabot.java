/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import org.usfirst.frc.team2077.commands.PrimaryStickDrive3Axis;
import org.usfirst.frc.team2077.commands.Rotate;
import org.usfirst.frc.team2077.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.MecanumChassis;
import org.usfirst.frc.team2077.drivetrain.PizzabotDriveModule;
import org.usfirst.frc.team2077.sensors.AnalogSettings;
import org.usfirst.frc.team2077.sensors.AngleSensor;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
public class Pizzabot extends Robot {

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
    DriveModuleIF[] pizzaDriveModules = {
      new PizzabotDriveModule(1, 10, 11, false),  // northeast (right front)
      new PizzabotDriveModule(2, 12, 13, false),  // southeast (right rear)
      new PizzabotDriveModule(3, 18, 19, true),  // southwest (left rear)
      new PizzabotDriveModule(4, 20, 21, true)   // northwest (left front)
    };
    /*
    DriveModule[] backwardPizzaDriveModules = { // Pizzabot with front/rear swapped
      new PizzabotDriveModule(3, 18, 19, false),  // southwest (left rear)
      new PizzabotDriveModule(4, 20, 21, false),   // northwest (left front)
      new PizzabotDriveModule(1, 10, 11, true),  // northeast (right front)
      new PizzabotDriveModule(2, 12, 13, true)  // southeast (right rear)
    };
    */
    /*
    DriveModule[] dummyMecanumDriveModules = {
      new DummyDriveModule(100),  // northeast (right front)
      new DummyDriveModule(100),  // southeast (right rear)
      new DummyDriveModule(100),  // southwest (left rear)
      new DummyDriveModule(100)   // northwest (left front)
    };
    */

    chassis_ = new MecanumChassis(pizzaDriveModules, constants_.PIZZABOT_WHEELBASE, constants_.PIZZABOT_TRACK_WIDTH, constants_.PIZZABOT_WHEEL_RADIUS); 
    
    // Subsystems.
    //   These dummy subsystems support sepearable command ownership of robot motion and rotation.
    position_ = new SubsystemBase() {};
    heading_ = new SubsystemBase() {};
    target_ = new SubsystemBase() {};

    //   Manages an operator-controlled aiming point displayed on DS and used for targeting.
    
    //First crosshairs is for double cam TODO: new focal length
    //crosshairs_ = new Crosshairs("Camera0", constants_.DOUBLE_CAMERA_PIXEL_WIDTH, constants_.DOUBLE_CAMERA_PIXEL_HEIGHT, 620,
      //constants_.DOUBLE_CAMERA_POSITION_NORTH, constants_.DOUBLE_CAMERA_HEIGHT, constants_.DOUBLE_CAMERA_TILT, constants_.MINI_TARGET_HEIGHT); 
   // crosshairs_ = new Crosshairs("Camera0", 360, 640, 620,
   //   constants_.PIZZABOT_CAMERA_POSITION_NORTH, constants_.PIZZABOT_CAMERA_HEIGHT, constants_.PIZZABOT_CAMERA_TILT, constants_.MINI_TARGET_HEIGHT); 

     // Initial robot and target position.
    //robot_.chassis_.setPosition(-78.25, 0, 0); // TODO: Way to set initial robot and target positions.
    //robot_.crosshairs_.setTarget(78.25, 0, 0); // TODO: Way to set initial robot and target positions.
    
    //System.out.println("CROSSHAIRS:" + crosshairs_);
    
    //   Ball launcher.
    //tlauncher_ = new TestLauncher();
    //(new ResetTarPos()).initialize();
    //tlauncher_ = new TestLauncher();
    //tgrabber_ = new TestGrabber();

    // Default teleop commands.
    //drive_ = new PrimaryStickDrive2Axis();
    drive_ = new PrimaryStickDrive3Axis();
    //aim_ = new AimCrosshairs();
    //track_ = new TrackTarget();
    //range_ = new RangeToCrosshairs(constants_.MINI_TARGET_HEIGHT - constants_.DOUBLE_CAMERA_HEIGHT);

    // Test code.
    //driveStation_.primary4_.whenPressed();
    //driveStation_.primary5_.whenPressed();  // TODO: debug
    //driveStation_.primary6_.whenPressed();
    //driveStation_.primary7_.whenPressed();  // TODO: debug
    //driveStation_.primary8_.whenPressed();
    //driveStation_.primary9_.whenPressed();  // TODO: debug
    //driveStation_.secondaryTrigger_.whenPressed(new ToggleLauncher());
    //driveStation_.secondary5_.whileHeld(new LoadLauncher());  // TODO: debug
    //driveStation_.secondary6_.whileHeld(new AngleLauncher());
    //driveStation_.primaryTrigger_.whileHeld(new RunGrabber());
    //driveStation_.primary8_.whenPressed(new MoveAngleTarget());
    //driveStation_.secondaryTrigger_.whileActiveContinuous(new ContinousAimToTarget3());
    //driveStation_.primary12_.whileHeld(new ToggleLauncher(), true);


    (new JoystickButton(driveStation_.primaryStick_, 7)).whenPressed(new Rotate(90));
    (new JoystickButton(driveStation_.primaryStick_, 8)).whenPressed(new Rotate(-90));
    (new JoystickButton(driveStation_.primaryStick_, 9)).whenPressed(new Rotate(360));
    (new JoystickButton(driveStation_.primaryStick_, 10)).whenPressed(new Rotate(-360));
    (new JoystickButton(driveStation_.primaryStick_, 11)).whenPressed(new Rotate(1080));
    (new JoystickButton(driveStation_.primaryStick_, 12)).whenPressed(new Rotate(-1080));
    

    //(new JoystickButton(driveStation_.secondaryStick_, 6)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVoltageTest());
    //(new JoystickButton(driveStation_.secondaryStick_, 7)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getRatioTest());
    //(new JoystickButton(driveStation_.secondaryStick_, 8)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getRevolutionsTest(10, .2));
    //(new JoystickButton(driveStation_.secondaryStick_, 6)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(32, .5));
    //(new JoystickButton(driveStation_.secondaryStick_, 7)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(16, 1));
    //(new JoystickButton(driveStation_.secondaryStick_, 8)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(8, 2));
    //(new JoystickButton(driveStation_.secondaryStick_, 9)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(4, 3));
    //(new JoystickButton(driveStation_.secondaryStick_, 10)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(2, 4));
    //(new JoystickButton(driveStation_.secondaryStick_, 11)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(1, 8));

    // TODO: There's supposedly another way to do this.
    CommandScheduler.getInstance().setDefaultCommand(position_, drive_);
    //CommandScheduler.getInstance().setDefaultCommand(target_, track_);
    // Uncomment the following to move rotation to secondary stick.
    //CommandScheduler.getInstance().setDefaultCommand(heading_, new SecondaryStickDrive());

    // Aiming system work in progress.
    //CommandScheduler.getInstance().setDefaultCommand(heading_, new SteerToCrosshairs());
    //CommandScheduler.getInstance().setDefaultCommand(crosshairs_, aim_);
    //CommandScheduler.getInstance().setDefaultCommand(tlauncher_, range_);
  }
}
