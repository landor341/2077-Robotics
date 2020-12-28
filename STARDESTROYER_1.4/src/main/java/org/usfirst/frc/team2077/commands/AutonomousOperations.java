// This file, as far as I can tell is only being worked on by me, therefore, currently, this is the file that you want to replace the old versions
package org.usfirst.frc.team2077.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc.team2077.subsystems.Launcher;//NEW

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutonomousOperations extends CommandBase{
///---Mapping---///
///---Mapping---///

double distanceWantedFromWall = 3;

String startingPosistion;
int startingNum;
double firstRotationToMove = 0; //0 is a default value
double firstDownX = 0; //0 is a default value
double firstWait = 0; //0 is a default value
double firstOverY = 0; //0 is a default value
double screwTo = 0; //0 is a default value

boolean feildMove;
boolean sectorMove;
boolean justMove;


String secoundMoveingPosistion;

//public void AutonomousOperations(){
    //}

    // public void startAutonomousOperations(){
    // }

    public void initialize(){}
// // TEMP FOR TABLES
//     // Launcher Launcher_ = new Launcher();
//     // Launcher_.AJNew();

//         // // First: Grab values from smartdashboard to choose path
//         // // Secound: After checking the screw's placement, launch all balls

//         // // setShuffleBoard();
//         setConditions();
//         checkShuffleBoard();
//         goArround();
//     }

    public void setShuffleBoard(){
        SmartDashboard.putNumber("Plan", 1);
        // SmartDashboard.getNumber("Plan", 1);
        SmartDashboard.putString("Starting: (A,B,C) = ", null);
        SmartDashboard.putString("Moveing: (L,M,R) = ", null);

        // Times
        SmartDashboard.putNumber("WaitingTime (sec)", 0);

        // Targets
        SmartDashboard.putBoolean("FeildMove  = ", false);
        SmartDashboard.putBoolean("SectorMove = ", false);
        SmartDashboard.putBoolean("JustMove   = ", false);
        // SmartDashboard.putBoolean("P3 = ", false);
        // SmartDashboard.putBoolean("P4 = ", false);
    }

    public void setConditions(){

        startingPosistion = SmartDashboard.getString("Starting: (L,M,R) = ", null).toUpperCase();
        secoundMoveingPosistion = SmartDashboard.getString("Moveing: (L,M,R) = ", null).toUpperCase();

        // Targets
        feildMove = SmartDashboard.getBoolean("FeildMove  = ", false);
        sectorMove = SmartDashboard.getBoolean("SectorMove = ", false);
        justMove = SmartDashboard.getBoolean("JustMove   = ", false);
        // p4 = SmartDashboard.getBoolean("P4 = ", false);
        firstWait = SmartDashboard.getNumber("WaitingTime (sec)", 0);

    }

    public void checkShuffleBoard(){
        if(startingPosistion.equals("A")){
            startingNum = 1;
            // robot_.tLauncher_.screwGoTo(3.0);
            firstRotationToMove = 13.2;//To get to being parralell to the side of the feild looking forward alighned relivitve to drive side
            firstDownX = 122.7 - distanceWantedFromWall;//122.655in
        }else if(startingPosistion.equals("B")){
            startingNum = 2;
            firstRotationToMove = 0; //In case the default value was not kept
            firstDownX = 94.7 - distanceWantedFromWall;//94.655in
        }else if(startingPosistion.equals("C")){
            startingNum = 3;
            firstRotationToMove = -13.2; //In case the default value was not kept
            firstDownX = 66.7 - distanceWantedFromWall;//66.655in

        } else {

        }

        // WAIT
        // getMatchTime

        

        // firstOverY is reduced by a few inches to account for the robot size
        if(feildMove){
            firstOverY = -72.57 - 5;
        } else if(sectorMove){
        firstOverY = 74;
        } else if(justMove){
            // Program special stopers for later here.
        } else {

        }
    }

// OPERATIONS:  Shoot all --> alignToTape --> YControls --> goToSide X + together --> 

    public void goArround(){
        // Check screw's placement and adjust it
        //////////////// use robot_.launcher_.setRangeUpper() ///////////// robot_.tLauncher_.screwGoTo(3.0); //Needs Calabration
    // Launch Balls     - Note: As of writing this, there is a load module created but it does not do anything yet.
        // robot_.launcher_.setRunning(true);
        // robot_.launcher_.load();
    // Wait For Mr.Shillings code is done launching TODO: Ask about launcher waiting time untill there are no balls present in the shooter?
        // robot_.launcher_.setRunning(false);

        if(sectorMove || feildMove || justMove){
            (new Move2(firstRotationToMove)).schedule(); //Alighment 90 degrees off of line
            // Mechaniums are now allong N/S route to be able to grab balls
            (new Move2(firstOverY, 0.0)).schedule(); //Move on Y
            (new Move2(90)).schedule(); //Alighment
            (new Move2(firstDownX, 0.0)).schedule(); //Already has distance from wall included
            (new Move2(90)).schedule();
            (new Move2(firstOverY + 86.57)).schedule();
// RUN Grabber HERE
            

            // (new Move2(firstDownX+10)).schedule(); // Added extra though-wall here
            // (new Move2(-10.0,0.0)).schedule(); // Backparking and need to change from useing distanceWantedFromWall
            // (new Move2(-10.0,0.0)).schedule();


        } else {
            (new Move2(firstRotationToMove + 90)).schedule(); //Alighment and so mechaniums are allong N/S route
            (new Move2(firstDownX)).schedule();
            (new Move2(90)).schedule();
            // Mechaniums are now allong N/S route to be able to grab balls
            (new Move2(86.57,0.0)).schedule();
        }
        // runGrabber

        
        // getToTrench
        



// goToSide - paused for now

    }

    public void justoview(){
        (new Move2(-24.0, 0.0)).schedule();         // Earn points for getting off of the tape
        (new Move2( 180.0 )).schedule();            // Turn forward
            // System.out.println("ERROR- Please enter starting values!!!");
    }



        // if(p1){ //   / by 8 because of space requirments
        //     // (new Move2(72.0,(startingNum * 80.8)/8)).schedule();//Value N1
        // }
    // }





    // public void launchFromLeft(){}
    // public void launchFromRight(){}


    // @Override
    // public void execute() {
    // }

}