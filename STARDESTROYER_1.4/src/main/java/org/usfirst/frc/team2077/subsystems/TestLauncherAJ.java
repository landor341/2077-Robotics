package org.usfirst.frc.team2077.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static org.usfirst.frc.team2077.Robot.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestLauncherAJ extends SubsystemBase implements LauncherIF {
    private final TalonSRX screw_;
    private final TalonSRX shooterL_;
    private final TalonSRX shooterR_;
    private final TalonSRX loader_;

    private boolean isLaunching;

    

    public final double launchMaxRPM = 4000;
    public final double maxPotVolt = 2.6;
    public final double minPotVolt = 0.1;

    
    private double[][] TestValues = {
        {-1, 0, 0},
        {50, 2, 200},
        {100, 2, 400},
        {150, 2, 600},
        {200, 2, 800},
        {250, 2, 1000},
        {9999, 5, launchMaxRPM}
    };
    
    private double preMotorSpeed = 0;

    private final double rightLeftBias = 0.0;    

    private final double unitsToRPM = (600. / 2048.);
    private final double kP = 0.1;
    private final double kI = 0.0001;
    private final double kD = 0.0;

    public TestLauncherAJ() {
        screw_ = new TalonSRX(3);
        screw_.configFactoryDefault();

        loader_ = new TalonSRX(6);
        loader_.configFactoryDefault();

        shooterL_ = new TalonSRX(4);
        shooterL_.configFactoryDefault();
        shooterL_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
        shooterL_.setSensorPhase(false);
        shooterL_.configNominalOutputForward(0, 0);
		shooterL_.configNominalOutputReverse(0, 0);
		shooterL_.configPeakOutputForward(1, 0);
        shooterL_.configPeakOutputReverse(-1, 0);
        shooterL_.config_kF(0, 0.0, 0);
		shooterL_.config_kP(0, kP, 0);
		shooterL_.config_kI(0, kI, 0);
		shooterL_.config_kD(0, kD, 0);

        shooterR_ = new TalonSRX(5);
        shooterR_.configFactoryDefault();
        shooterR_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
        shooterR_.setSensorPhase(false);
        shooterR_.configNominalOutputForward(0, 0);
		shooterR_.configNominalOutputReverse(0, 0);
		shooterR_.configPeakOutputForward(1, 0);
		shooterR_.configPeakOutputReverse(-1, 0);
        shooterR_.config_kF(0, 0.0, 0);
		shooterR_.config_kP(0, kP, 0);
		shooterR_.config_kI(0, kI, 0);
		shooterR_.config_kD(0, kD, 0);

        isLaunching = false;
    }

    public void runAngle(boolean dir) {
        if(dir) {
            screw_.set(ControlMode.PercentOutput, 1.0);
        } else {
            screw_.set(ControlMode.PercentOutput, -1.0);
        }
        
    }

    public void stopAngle() {
        screw_.set(ControlMode.PercentOutput, 0.0);
    }

    public void runLauncher(double velocity) {
        shooterL_.set(ControlMode.Velocity,velocity/unitsToRPM);
        shooterR_.set(ControlMode.Velocity,velocity/unitsToRPM);
    }

    public void stopLauncher() {
        shooterL_.set(ControlMode.PercentOutput,0.0);
        shooterR_.set(ControlMode.PercentOutput,0.0);
    }

    public void toggleLauncher(double velocity) {
        if(isLaunching) {
            shooterL_.set(ControlMode.PercentOutput,0.0);
            shooterR_.set(ControlMode.PercentOutput,0.0);
            isLaunching = false;
        } else {
            shooterL_.set(ControlMode.Velocity,velocity/unitsToRPM);
            shooterR_.set(ControlMode.Velocity,velocity*(1-rightLeftBias)/unitsToRPM);
            isLaunching = true;
        }
    }

    public double getLaunchVelL() {
        double res = shooterL_.getSelectedSensorVelocity(0) * unitsToRPM;
        
        return res;
    }

    public double getLaunchVelR() {
        return shooterR_.getSelectedSensorVelocity(0) * unitsToRPM;
    }

    public void runLoader(double speed) {
        loader_.set(ControlMode.PercentOutput,speed);
    }

    public void stopLoader() {
        loader_.set(ControlMode.PercentOutput,0.0);
    }
   
    public void stopAll() {
        shooterL_.set(ControlMode.PercentOutput,0.0);
        shooterR_.set(ControlMode.PercentOutput,0.0);
        loader_.set(ControlMode.PercentOutput,0.0);
        screw_.set(ControlMode.PercentOutput,0.0);
    }

    public boolean setRangeLower(double range) {
        return false;
    }
    public boolean setRangeUpper(double range) {
        boolean done = false;
        int i = -1;
        while (!done) {
            i++;
            if (TestValues[i][0] > range) {
                done = true;
            }
        }
        //runLauncher(TestValues[i][2]);
        screwGoTo(TestValues[i][1]);
        runLauncher(TestValues[i][2]);

        return true;
    }

    public void screwGoTo(double goalVoltage_){ //NEEDS CHECK
        final double powerValue_ = 0.9;//TODO: move upwards
        final double maxVoltageLimit_ = 2.6;
        final double minVoltageLimit_ = 0.1;
    // Goal voltage within paramaters check
        if(goalVoltage_ > maxVoltageLimit_){
            goalVoltage_ = maxVoltageLimit_;
        } else if(goalVoltage_ < minVoltageLimit_){
            goalVoltage_ = minVoltageLimit_;
        }
        double currentScrewVoltage_ = robot_.potentialSensor_.getScrewVoltage();

        // while(currentScrewVoltage_ != goalVoltage_){
        if (goalVoltage_ > currentScrewVoltage_) {
            while(currentScrewVoltage_ < goalVoltage_ && currentScrewVoltage_ < maxVoltageLimit_){
                System.out.println("NEGATIVE POWER ****************");
                if(currentScrewVoltage_ < goalVoltage_){
                    screw_.set(ControlMode.PercentOutput, powerValue_);
                } 
                currentScrewVoltage_ = robot_.potentialSensor_.getScrewVoltage();
            }
        } else if (goalVoltage_ > currentScrewVoltage_) {
            while(currentScrewVoltage_ > goalVoltage_ && currentScrewVoltage_ > minVoltageLimit_){
                System.out.println("NEGATIVE POWER ****************");
                if(currentScrewVoltage_ > goalVoltage_){
                    screw_.set(ControlMode.PercentOutput, -powerValue_);
                }
                currentScrewVoltage_ = robot_.potentialSensor_.getScrewVoltage();
            }
        }
        screw_.set(ControlMode.PercentOutput,0.0);
    }

    public void screwSet(double giveVoltageValue_){
        final double maxVoltageLimit_ = 2.6;
        final double minVoltageLimit_ = 0.1;
        if(robot_.potentialSensor_.getScrewVoltage() > maxVoltageLimit_) {
            if (giveVoltageValue_ < 0) {
                screw_.set(ControlMode.PercentOutput,giveVoltageValue_);
            }
        } else if (robot_.potentialSensor_.getScrewVoltage() < minVoltageLimit_){
            if (giveVoltageValue_ > 0) {
                screw_.set(ControlMode.PercentOutput,giveVoltageValue_);
            }
        }else if (robot_.potentialSensor_.getScrewVoltage() > minVoltageLimit_ && robot_.potentialSensor_.getScrewVoltage() < maxVoltageLimit_) {
            screw_.set(ControlMode.PercentOutput,giveVoltageValue_);
        } else {
            screw_.set(ControlMode.PercentOutput,0.0);
        }

    }


// AJ's New Modules Located Here
    public void printInfraredReading(){
        System.out.println(robot_.infraredSensor_.getInfraredVoltage());
    }

    public boolean isLoaded() {// <--This is a replacement for a empty module mentioned next, just used to keep old code working, acts as a forwarding
        return robot_.infraredSensor_.getStatusIsLoaded();
    }

// AJ's New Modules Stopped Here


    // public boolean isLoaded() {
    //     return false;
    // }

    public void launch() {
    }

    public String toString() {
        return "";
    }

    @Override
    public void setRunning(boolean running) {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isReady() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean isRunning() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void load() {
        // TODO Auto-generated method stub

    }
}
