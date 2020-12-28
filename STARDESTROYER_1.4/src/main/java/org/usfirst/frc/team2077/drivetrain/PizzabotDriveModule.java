/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

public class PizzabotDriveModule implements DriveModuleIF {

    private static final double SECONDS_PER_MINUTE = 60.;

    private static final double SRX_SENSOR_UNITS_PER_SECOND = 10.;
    private static final int ENCODER_PULSES_PER_REVOLUTION = 2048; // pulses per revolution (for CUI AMT103 this is the default switch setting per their data sheet)
    private static final int ENCODER_COUNTS_PER_REVOLUTION = 4 * ENCODER_PULSES_PER_REVOLUTION; 

    private static final double MOTOR_RPM_LIMIT = robot_.constants_.PIZZABOT_MOTOR_RPM_LIMIT;

    // TODO: Take the constants below as parameters to a base SrxCimDriveModule
    private static final double GEAR_RATIO = 21.25; // gear reduction between motor and wheel (Pizzabot) // TODO: check this
    private static final boolean INVERTING_GEARBOX = false; // true if wheel direction is opposite motor direction
    private static final double WHEEL_RADIUS = 3.; // inches (Pizzabot)

    // velocity limit
    private final double maximumVelocity_ = (MOTOR_RPM_LIMIT/GEAR_RATIO) / (SECONDS_PER_MINUTE / (2 * Math.PI * WHEEL_RADIUS));

    private final WPI_TalonSRX  talonSRX_;

    private final Encoder wheelEncoder_;

    private final boolean reversed_;


    // TODO: Decide where to put these.
    private static final double motorKP = 0.030;
    private static final double motorKI = 0.100;
    private static final double motorKD = 0.000;
    private static final double motorKF = 0.015;

    // velocity set point
    private double velocity_ = 0;

    // accumulated distance since last velocity change or distance reset
    //private double distanceIntegratedFromSetVelocity_ = 0;
    //private double distanceIntegratedFromGetVelocity_ = 0;

    // time of last distance reset
    private double distanceTime_ = Timer.getFPGATimestamp();

    // Pizzabot configuration of an AMT103 encoder on the motor shaft connected to the SRX,
    // and a second AMT103 on the wheel axle connected to two DIO channels.
    public PizzabotDriveModule(int canID, int wheelEncoderChannelA, int wheelEncoderChannelB, boolean reversed) {

        talonSRX_ = new WPI_TalonSRX(canID);
        talonSRX_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0); // TODO: return code? timeout value?
        talonSRX_.configSelectedFeedbackCoefficient(1.0, 0, 0); // TODO: return code? timeout value?
        talonSRX_.setSensorPhase(INVERTING_GEARBOX);
        talonSRX_.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 0); // TODO: frame? timeout value?
        talonSRX_.config_kP(0, motorKP, 0);
        talonSRX_.config_kI(0, motorKI, 0);
        talonSRX_.config_kD(0, motorKD, 0);
        talonSRX_.config_kF(0, motorKF, 0);
        talonSRX_.configMaxIntegralAccumulator(0, 300, 0); // TODO: ????????????
        talonSRX_.configClosedloopRamp(0.2, 0);  // .2 seconds from neutral to full
        talonSRX_.setNeutralMode(NeutralMode.Brake);

        wheelEncoder_ = new Encoder(new DigitalInput(wheelEncoderChannelA), new DigitalInput(wheelEncoderChannelB), false, EncodingType.k4X);
        wheelEncoder_.setDistancePerPulse(4); // convert to encoder counts

        reversed_ = reversed;
    }

    @Override
    public double getMaximumSpeed() {
        return maximumVelocity_;
    }

    @Override
    public void setVelocity(double velocity) {

        //double now = Timer.getFPGATimestamp();
        //distanceIntegratedFromSetVelocity_ += velocity_ * (now-distanceTime_);
        //distanceIntegratedFromGetVelocity_ += getVelocity() * (now-distanceTime_);
        //distanceTime_ = now;

        velocity_ = velocity; // in inches/second

        double rps = velocity_ / (2 * Math.PI * WHEEL_RADIUS); // wheel revolutions/second
        rps *= GEAR_RATIO; // motor revolutions/second
        double setPoint = rps * ENCODER_COUNTS_PER_REVOLUTION / SRX_SENSOR_UNITS_PER_SECOND; // set point is in counts/100ms
        
        talonSRX_.set(ControlMode.Velocity, setPoint * (reversed_ ? -1 : 1));
    }

    @Override
    public double getVelocity() {
        
        // return velocity_; // return the set point

        double rps = talonSRX_.getSensorCollection().getQuadratureVelocity() * SRX_SENSOR_UNITS_PER_SECOND / ENCODER_COUNTS_PER_REVOLUTION;
        rps /= GEAR_RATIO; // wheel revolutions/second
        double ips = rps * (2. * Math.PI * WHEEL_RADIUS);
        double velocity = ips * (reversed_ ? -1 : 1);

        return velocity;
    }

    @Override
    public double getDistance() {

        //double now = Timer.getFPGATimestamp();
        //return distanceIntegratedFromSetVelocity_ + velocity_ * (now-distanceTime_); // return the distance calculated by integrating velocity set point over time
        //return distanceIntegratedFromGetVelocity_ + getVelocity() * (now-distanceTime_); // return the distance calculated by integrating velocity over time
        
        double revolutions = getWheelRevolutions();
        double inches = revolutions * (2. * Math.PI * WHEEL_RADIUS);

        return inches * (reversed_ ? -1 : 1);
    }

    @Override
    public void resetDistance() {

        //distanceIntegratedFromSetVelocity_ = 0;
        //distanceIntegratedFromGetVelocity_ = 0;
        //distanceTime_ = Timer.getFPGATimestamp();

        wheelEncoder_.reset();
    }

    @Override
    public String toString() {
        return "" + Math.round(velocity_*10.)/10. + "(" + Math.round(getVelocity()*10.)/10. + ")";
    }

    private double getWheelRPM() {
        // getRate() returns counts per second
        return wheelEncoder_.getRate() * SECONDS_PER_MINUTE / ENCODER_COUNTS_PER_REVOLUTION;
    }

    private double getMotorRPM() {
        // getQuadratureVelocity() returns counts per 100ms.
        return talonSRX_.getSensorCollection().getQuadratureVelocity() * SRX_SENSOR_UNITS_PER_SECOND * SECONDS_PER_MINUTE / ENCODER_COUNTS_PER_REVOLUTION;
    }

    private void resetWheelRevolutions() {
        wheelEncoder_.reset();
    }

    private void resetMotorRevolutions() {
        talonSRX_.getSensorCollection().setQuadraturePosition(0, 0);
    }

    private double getWheelRevolutions() {
        return wheelEncoder_.getDistance() / ENCODER_COUNTS_PER_REVOLUTION;
    }

    private double getMotorRevolutions() {
        return talonSRX_.getSensorCollection().getQuadraturePosition() / ENCODER_COUNTS_PER_REVOLUTION;
    }

    public Command getVoltageTest() {
        return new TestVoltage(8, 2);
    }
    public Command getVoltageTest(double cyclePeriod, double cycleCount) {
        return new TestVoltage(cyclePeriod, cycleCount);
    }

    public Command getVelocityTest() {
        return new TestVelocity(8, 2);
    }
    public Command getVelocityTest(double cyclePeriod, double cycleCount) {
        return new TestVelocity(cyclePeriod, cycleCount);
    }

    public Command getRatioTest() {
        return new TestRatio(8, 2);
    }
    public Command getRatioTest(double cyclePeriod, double cycleCount) {
        return new TestRatio(cyclePeriod, cycleCount);
    }

    public Command getRevolutionsTest(double revolutions, double velocity01) {
        return new TestRevolutions(revolutions, velocity01);
    }

    private class Median3 {

        double[] sample_ = {0, 0, 0};
        double[] scratch_ = new double[3];

        int index_ = 0;

        double filter(double sample) {
            sample_[index_] = sample;
            index_ = (index_+1) % 3;
            System.arraycopy(sample_, 0, scratch_, 0, 3);
            Arrays.sort(scratch_);
            return scratch_[1];
        }
    }

    public class TestVoltage extends CommandBase {

        private final double cyclePeriod_;
        private final double cycleCount_;

        private double startTime_;
        private double endTime_;

        private double setPoint_ = 0;

        private Median3 wheelFilter_ = new Median3();
        private Median3 motorFilter_ = new Median3();

        private double wheelMin_ = 0;
        private double wheelMax_ = 0;
        private double motorMin_ = 0;
        private double motorMax_ = 0;

        public TestVoltage(double cyclePeriod, double cycleCount) {
            System.out.println("================ TestVoltage(" + cyclePeriod + ", " + cycleCount + ") " + this);
            cyclePeriod_ = cyclePeriod;
            cycleCount_ = cycleCount;
        }
        
        @Override
        public void initialize() {
            System.out.println("================ initialize() " + this);

            startTime_ = Timer.getFPGATimestamp();
            endTime_ = startTime_ + cycleCount_ * cyclePeriod_;
        }
        
        @Override
        public void execute() {

            //double wheelRPM = wheelFilter_.filter(getWheelRPM()) * 21.42;
            double wheelRPM = wheelFilter_.filter(getWheelRPM());
            double motorRPM = motorFilter_.filter(getMotorRPM());
            double voltage = Math.round(talonSRX_.getBusVoltage()*10.)/10.;

            wheelMin_ = Math.min(wheelRPM, wheelMin_);
            wheelMax_ = Math.max(wheelRPM, wheelMax_);
            motorMin_ = Math.min(motorRPM, motorMin_);
            motorMax_ = Math.max(motorRPM, motorMax_);

            System.out.println("================ execute()"
                + " " + voltage
                + " " + (int)Math.round(wheelMin_)
                + " " + (int)Math.round(motorMin_)
                + " " + (int)Math.round(wheelMax_)
                + " " + (int)Math.round(motorMax_)
                + " " + (int)Math.round(wheelRPM)
                + " " + (int)Math.round(motorRPM)
                + " " + Math.round(setPoint_*1000.)/1000.);

            double elapsed = Timer.getFPGATimestamp() - startTime_;

            double now = ( elapsed % cyclePeriod_ ) / cyclePeriod_; // 0.0 - 1.0
        
            double phase = 4. * Math.PI * now;
            double sign = now < .5 ? 1 : -1;
        
            setPoint_ = ( -Math.cos(phase) / 2 + .5 ) * sign;  // -1.0 - 0.0 - 1.0, sinusoidal approaches to min, max, and zero
           
            talonSRX_.set(ControlMode.PercentOutput, setPoint_);
        }
        
        @Override
        public void end(boolean interrupted) {
            System.out.println("================ end(" + interrupted + ")"
                + " " + (int)Math.round(wheelMin_)
                + " " + (int)Math.round(motorMin_)
                + " " + (int)Math.round(wheelMax_)
                + " " + (int)Math.round(motorMax_));
        }
        
        @Override
        public boolean isFinished() {
            return Timer.getFPGATimestamp() > endTime_;
        }
    }

    public class TestVelocity extends CommandBase {

        private long n_ = 0;

        private final double cyclePeriod_;
        private final double cycleCount_;

        private double startTime_;
        private double endTime_;

        private double setPoint_ = 0;

        private Median3 wheelFilter_ = new Median3();
        private Median3 motorFilter_ = new Median3();

        private double wheelMin_ = 0;
        private double wheelMax_ = 0;
        private double motorMin_ = 0;
        private double motorMax_ = 0;

        public TestVelocity(double cyclePeriod, double cycleCount) {
            System.out.println("================ TestVelocity(" + cyclePeriod + ", " + cycleCount + ") " + this);
            cyclePeriod_ = cyclePeriod;
            cycleCount_ = cycleCount;
        }
        
        @Override
        public void initialize() {
            System.out.println("================ initialize() " + this);

            startTime_ = Timer.getFPGATimestamp();
            endTime_ = startTime_ + cycleCount_ * cyclePeriod_;
        }
        
        @Override
        public void execute() {

            //double wheelRPM = wheelFilter_.filter(getWheelRPM()) * 21.42;
            double wheelRPM = wheelFilter_.filter(getWheelRPM());
            double motorRPM = motorFilter_.filter(getMotorRPM());
            double voltage = Math.round(talonSRX_.getBusVoltage()*10.)/10.;

            wheelMin_ = Math.min(wheelRPM, wheelMin_);
            wheelMax_ = Math.max(wheelRPM, wheelMax_);
            motorMin_ = Math.min(motorRPM, motorMin_);
            motorMax_ = Math.max(motorRPM, motorMax_);
            
            double velocity = talonSRX_.getSensorCollection().getQuadratureVelocity();

            if ((n_++ % 10) == 0)
            System.out.println("================ execute()"
                //+ " " + voltage
                //+ " " + (int)Math.round(wheelMin_)
                //+ " " + (int)Math.round(motorMin_)
                //+ " " + (int)Math.round(wheelMax_)
                //+ " " + (int)Math.round(motorMax_)
                //+ " " + (int)Math.round(wheelRPM)
                //+ " " + (int)Math.round(motorRPM)
                + " " + Math.round(velocity*100.)/100.
                + " " + Math.round(setPoint_*100.)/100.);

            double elapsed = Timer.getFPGATimestamp() - startTime_;

            double now = ( elapsed % cyclePeriod_ ) / cyclePeriod_; // 0.0 - 1.0
        
            double phase = 4. * Math.PI * now;
            double sign = now < .5 ? 1 : -1;
        
            setPoint_ = ( -Math.cos(phase) / 2 + .5 ) * sign; // -1.0 - 0.0 - 1.0, sinusoidal approaches to min, max, and zero

            double max = ENCODER_COUNTS_PER_REVOLUTION / (SRX_SENSOR_UNITS_PER_SECOND * SECONDS_PER_MINUTE) * MOTOR_RPM_LIMIT;
            setPoint_ *= max;
           
            //talonSRX_.set(ControlMode.PercentOutput, setPoint_);
            talonSRX_.set(ControlMode.Velocity, setPoint_);
        }
        
        @Override
        public void end(boolean interrupted) {
            System.out.println("================ end(" + interrupted + ")"
                + " " + (int)Math.round(wheelMin_)
                + " " + (int)Math.round(motorMin_)
                + " " + (int)Math.round(wheelMax_)
                + " " + (int)Math.round(motorMax_));
        }
        
        @Override
        public boolean isFinished() {
            return Timer.getFPGATimestamp() > endTime_;
        }
    }

    public class TestRatio extends CommandBase {

        private final double cyclePeriod_;
        private final double cycleCount_;

        private double startTime_;
        private double endTime_;

        private double wheelRevolutions_;
        private double motorRevolutions_;
        private double wheelRevolutions;
        private double motorRevolutions;

        private double setPoint_ = 0;
        private double setSign_ = 0;

        public TestRatio(double cyclePeriod, double cycleCount) {
            System.out.println("================ TestRatio(" + cyclePeriod + ", " + cycleCount + ") " + this);
            cyclePeriod_ = cyclePeriod;
            cycleCount_ = cycleCount;
        }
        
        @Override
        public void initialize() {
            System.out.println("================ initialize() " + this);

            startTime_ = Timer.getFPGATimestamp();
            endTime_ = startTime_ + cycleCount_ * cyclePeriod_;

            wheelRevolutions_ = 0;
            motorRevolutions_ = 0;
            wheelRevolutions = 0;
            motorRevolutions = 0;
    

            resetWheelRevolutions();
            resetMotorRevolutions();
        }
        
        @Override
        public void execute() {

            //double wheelRevolutions = getWheelRevolutions();
            //double motorRevolutions = getMotorRevolutions();
            wheelRevolutions = getWheelRevolutions();
            motorRevolutions = getMotorRevolutions();
            
            double voltage = Math.round(talonSRX_.getBusVoltage()*10.)/10.;

            System.out.println("================ execute()"
                + " " + voltage
                + " " + (int)Math.round(wheelRevolutions)
                + " " + (int)Math.round(motorRevolutions)
                + " " + (int)Math.round(wheelRevolutions_ + Math.abs(wheelRevolutions))
                + " " + (int)Math.round(motorRevolutions_ + Math.abs(motorRevolutions))
               + " " + Math.round(setPoint_*1000.)/1000.);

            double elapsed = Timer.getFPGATimestamp() - startTime_;

            double now = ( elapsed % cyclePeriod_ ) / cyclePeriod_; // 0.0 - 1.0
        
            double phase = 4. * Math.PI * now;
            double sign = now < .5 ? 1 : -1;
        
            setPoint_ = ( -Math.cos(phase) / 2 + .5 ) * sign; // -1.0 - 0.0 - 1.0, sinusoidal approaches to min, max, and zero

            if (Math.signum(setPoint_) != setSign_) {
                wheelRevolutions_ += Math.abs(wheelRevolutions);
                motorRevolutions_ += Math.abs(motorRevolutions);
                resetWheelRevolutions();
                resetMotorRevolutions();
                setSign_ = Math.signum(setPoint_);
            }
           
            talonSRX_.set(ControlMode.PercentOutput, setPoint_);
        }
        
        @Override
        public void end(boolean interrupted) {
            System.out.println("================ end(" + interrupted + ")"
            + " " + (int)Math.round(wheelRevolutions_)
            + " " + (int)Math.round(motorRevolutions_)
            + " " + Math.round(100.*motorRevolutions_/wheelRevolutions_)/100.);
        }
        
        @Override
        public boolean isFinished() {
            return Timer.getFPGATimestamp() > endTime_;
        }
    }

    public class TestRevolutions extends CommandBase {

        private final double revolutions_;
        private final double setPoint_;

        private Median3 wheelFilter_ = new Median3();
        private Median3 motorFilter_ = new Median3();

        private double setSign_ = 0;

        public TestRevolutions(double revolutions, double setPoint) {
            System.out.println("================ TestRevolutions(" + revolutions + ", " + setPoint + ") " + this);
            revolutions_ = revolutions;
            setPoint_ = setPoint;
        }
        
        @Override
        public void initialize() {
            System.out.println("================ initialize() " + this);

            resetWheelRevolutions();
            resetMotorRevolutions();
            }
        
        @Override
        public void execute() {

            double wheelRPM = wheelFilter_.filter(getWheelRPM());
            double motorRPM = motorFilter_.filter(getMotorRPM());
            double wheelRevolutions_ = getWheelRevolutions();
            double motorRevolutions_ = getMotorRevolutions();
             
            double voltage = Math.round(talonSRX_.getBusVoltage()*10.)/10.;

            System.out.println("================ execute()"
                + " " + voltage
                + " " + Math.round(wheelRevolutions_*10.)/10.
                + " " + Math.round(motorRevolutions_*10.)/10.
                + " " + Math.round(wheelRPM*10.)/10.
                + " " + Math.round(motorRPM*10.)/10.
                + " " + Math.round(setPoint_*1000.)/1000.);

            talonSRX_.set(ControlMode.PercentOutput, setPoint_);
        }
        
        @Override
        public void end(boolean interrupted) {
            System.out.println("================ end(" + interrupted + ")"
                + " " + Math.round(getWheelRevolutions()*10.)/10.
                + " " + Math.round(getMotorRevolutions()*10.)/10.);

            talonSRX_.set(ControlMode.PercentOutput, 0);
        }
        
        @Override
        public boolean isFinished() {
            return Math.abs(getWheelRevolutions()) >= Math.abs(revolutions_);
        }
    }
}