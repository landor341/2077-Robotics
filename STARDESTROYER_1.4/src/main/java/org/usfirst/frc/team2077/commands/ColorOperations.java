package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj.I2C;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Talon;//For the motor control

import edu.wpi.first.wpilibj.DriverStation;


// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.TimedRobot;             //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// public class ColorWithMoveWheel extends TimedRobot{  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

public class ColorOperations extends CommandBase{
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final int[] wheelHue = {0, 90, 120, 180, 360}; // need empirical tweaking?
    // private final String[] wheelColor = {"red", "yellow", "green", "blue", "red"};

    String gameData; //Came with examplecode 
    int colorNum; //The stored number from the color given by the Game Data VI (nothing/error = 0, Blue = 1, Green =2, Red = 3, Yellow = 4)
    int currentColorNum; //The value the sensor detects the robot is currently on
    // static double distancePerColor = 1;
    // static int defaultTime = 1;//This is the ammount of time it takes per color it needs to pass
    // int setPlaceholdeForMotor = 0; //This is a placeholder for where the direct run motor will be
    // Talon robot_.colorTalonMotor_ = new Talon(9);//Initializes a Motor on PWM #

    int numberOfSectorsPassed = 0;
    int neededNumberOfSectorsPassed = 24;// 24;// = setPlaceholdeForMotor;
    int lastColorNumber = findCurrentColorNum();

    int stoppingColorNum = 0; //For overshoot, 0 = default value

    // double defaultSetForColorWheel = -0.4;
    // double defaultSetForColorWheel = -0.7;
    double defaultSetForColorWheel = -0.95;
    double secoundSetValue_ = -0.5;
    // double defaultSetForColorWheel = 0.2;

    public ColorOperations() {
        addRequirements(robot_.colorWheel_);
      }
    
    @Override
    public void initialize(){
        doEverything();
    }

    @Override
    public void end(boolean interrupted) {
    }
 
    @Override
    public boolean isFinished() {
      return true;
    }



    public void doEverything(){
        getSentColorNum();
        moveColorMotorToSpot();
    }

    public int getSentColorNum(){
        // colorNum = 1;//<<
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        currentColorNum = findCurrentColorNum();
        if(gameData.length() > 0){
        switch (gameData.charAt(0))
            // B=4,G=3,R=1,Y=2
        {
            case 'B' :
                colorNum = 4;
                stoppingColorNum = 3;
            break;
            case 'G' :
                colorNum = 3;
                stoppingColorNum = 1;
            break;
            case 'R' :
                colorNum = 1;
                stoppingColorNum = 2;
            break;
            case 'Y' :
                colorNum = 2;
                stoppingColorNum = 4;
            break;
            default :
                colorNum = 0;
            break;
        }
        }else{//Code for no data received yet
        System.out.println("ERROR: colorNum is spesificaly set to 1, also, yes I know I am bad at spelling");
        }

        // System.out.println("===========================================================");
        // System.out.println("The Required Color Number To End On Is " + colorNum);
        // System.out.println("setPlaceholderForMotor = " + setPlaceholdeForMotor);
        // System.out.println("gameData = " + gameData);
        // System.out.println("currentColorNum = " + currentColorNum);
        // System.out.println("");
        // System.out.println("===========================================================");
        return colorNum;
    }


    public int findCurrentColorNum(){

        final ColorSensorV3.RawColor rawColor  = m_colorSensor.getRawColor();
        //System.out.println("" + rawColor.red + "\t" + rawColor.green +"\t" + rawColor.blue);
        // convert raw color to HSB
        final double divisor = Math.max(rawColor.red,Math.max(rawColor.green,rawColor.blue));
        final float[] hsb = java.awt.Color.RGBtoHSB((int)(rawColor.red*255./divisor), (int)(rawColor.green*255./divisor), (int)(rawColor.blue*255./divisor), null);
        final int hue = (int)Math.round(360*hsb[0]); // integer hue 0 - 360
        // SmartDashboard.putNumber("Hue", hue);

        // compare measured hue with wheel colors, picking the closest
        int proximity = Integer.MAX_VALUE;
        int index = 0;
        for (int i = 0; i < wheelHue.length; i++) {
        final int p = Math.abs(hue - wheelHue[i]);
        if (p < proximity) {
            proximity = p;
            index = i;
            }
        }  
        //   SmartDashboard.putString("Color", wheelColor[index]);
        // System.out.println("HUE:" + hue + "\t" + "COLOR:" + wheelColor[index]);
        // System.out.println("END OF ColorWith...");

        if(index == 4){//Convert to just color numbers.
            index = 0;
        }
        index++;
        return index;
    }

    public void moveColorMotorToSpot(){//int desiredColorNum_){
        System.out.println();
        // B=4,G=3,R=1,Y=2
        
            robot_.colorWheel_.talon_.set(defaultSetForColorWheel);

        while(numberOfSectorsPassed < neededNumberOfSectorsPassed){
            // System.out.println(findCurrentColorNum());
            
        
            // robot_.colorTalonMotor_.set(defaultSetForColorWheel);
            /// System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);
                if(lastColorNumber != findCurrentColorNum()){
                    if(lastColorNumber == 4){
                        if(findCurrentColorNum() == 2){
                            lastColorNumber = findCurrentColorNum();
                            numberOfSectorsPassed++;
                            System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);

                        } 
                    } else if(lastColorNumber == 1 || lastColorNumber == 5){
                        if(findCurrentColorNum() == 3){
                            lastColorNumber = findCurrentColorNum();
                            numberOfSectorsPassed++;
                            System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);

                        }
                    } else {
                        lastColorNumber = findCurrentColorNum();
                        numberOfSectorsPassed++;
                        System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);

                    }
                }
            }
        System.out.println("");
        System.out.println("Searching for color num of " + colorNum);

        //robot_.colorTalonMotor_.set(-0.4);//defaultSetForColorWheel + 0.7);
        robot_.colorWheel_.talon_.set(secoundSetValue_);//defaultSetForColorWheel + 0.7);
        // lastColorNumber = findCurrentColorNum();
        stoppingColorNum = colorNum;
        while(stoppingColorNum != lastColorNumber){
        // while(stoppingColorNum != lastColorNumber){
                // while(movementForOvershoot != findCurrentColorNum()){
            // while(getSentColorNum() != lastColorNumber){
                


            // while(getSentColorNum() != lastColorNumber){

            // robot_.colorTalonMotor_.set(defaultSetForColorWheel - 0.05);
            /// System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);
                if(lastColorNumber != findCurrentColorNum()){
                    if(lastColorNumber == 4){
                        if(findCurrentColorNum() == 2){
                            lastColorNumber = findCurrentColorNum();
                            numberOfSectorsPassed++;
                            System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);

                        } 
                    } else if(lastColorNumber == 1 || lastColorNumber == 5){
                        if(findCurrentColorNum() == 3){
                            lastColorNumber = findCurrentColorNum();
                            numberOfSectorsPassed++;
                            System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);

                        }
                    } else {
                        lastColorNumber = findCurrentColorNum();
                        numberOfSectorsPassed++;
                        System.out.println("numberOfSectorsPassed = "+numberOfSectorsPassed +" \t currentColorNum = "+findCurrentColorNum()+" \t lastColorNumber = "+lastColorNumber);

                    }
                }
            // }
        }
        robot_.colorWheel_.talon_.set(0);
        System.out.println("TASK COMPLEATED");

        numberOfSectorsPassed = 0;


///
        //   if(colorWheelObject.findCurrentColorNum() == colorGoal){
        //     colorMotor.set(0);
        //     System.out.println("Desired color reached");
        //   } else {
        //     colorMotor.set(0.4);
        //     System.out.println("curretColor = "+colorWheelObject.findCurrentColorNum()+" \t colorGoal = " + colorGoal);
    
        //   }
        // } else {
        //   colorMotor.set(0);
        // }
///
        // return 0;
    }

}