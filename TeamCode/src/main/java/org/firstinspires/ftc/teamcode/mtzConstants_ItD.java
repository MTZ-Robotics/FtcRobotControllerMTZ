package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/***********************************************
 * Constants for use in Autonomous or TeleOp
 *
 * Versions
 * v000 Saved after 08Jan2022 Competition
 * v001 Changes after 08Jan2022 Competition
 * v002 Changes after 22Jan2022 Competition
 * v201 Changes for arm with new shoulder and extension and double claw
 * v202 Corrections on 01Feb2022
 * v310
 * v302 Added wrist adjustment
 *
 ***********************************************/
public class mtzConstants_ItD {
// Adjustments for efficiency
    public static final double driveEfficiency = .868;
    public static final double strafeEfficiency = 1.15;
    public static final double turnEfficiency = 62;
    public static final double armRotationEfficiency = 1.0;
    public static final double armExtensionEfficiency = 1.0;
// Debug Delay
public static final int defaultPauseTime = 100;  //Milliseconds after a command
    public static final int earlyDelayPauseTime = 10000;  //Milliseconds to wait for other robots to pass the area



// Timer
    public static double endGameStart = 90;
    public static double endGameWarning = endGameStart + 15;
    public static double endGameWarning2 = endGameStart + 25;
    public static double endGameOver = endGameStart + 30;
    public static double greenWarningTime = 60;
    public static double yellowWarningTime = 70;
    public static double redWarningTime = 80;

// Powers & Speeds

    //public static double defaultArmPower = 0.2;
    public static double defaultArmPower = 0.5;
    public static double defaultArmLowerPower = 0.2;
    //arm assist was 0.020, but that was with a heavier claw. need to adjust this to the current claw
    //arm assist value before making changes: 0.25. Hope changed this to a much lower value
    //because the default arm power seems to have been turned up, so the arm assist was becoming a hindrance by constantly moving upward.
    public static double defaultArmAssistLevel = 0.005;
    public static double armAssistLevelLoaded = defaultArmAssistLevel + 0.014;
    public static double defaultArmExtensionPower = 1.0;

    public static double driveBump = 1; // inches
    public static double strafeBump = 1; // inches
    public static double turnBump = 3; // degrees
    public static double wristBump = 0.01; // servo
    public static double wristAdjustment = 0.00; // servo Correction

    public static int scoopStage;

    public static double defaultDriveSpeed = 0.35;
    public static double driveSlowRatio = 0.35;
    public static double driveFastRatio = 1/0.7;

    public static double defaultIntakeSpeed = 0.25;
    public static double defaultFlywheelSpeed = 0.08;

    //Align to AprilTag Variables
    public static double alignConfidence = 0.001;
    // Adjust these numbers to suit your robot.
    public static double backdropAprilTagDESIRED_DISTANCE = 9; //  this is how close the camera should get to the target (inches)
    public static double cameraBearingOffsetLeftTagLeftPixelLeftSide = -12; //this is how far to the left the tag should be from the camera for the robot to drop the left pixel on the left side of the mountain
    public static double cameraBearingOffsetRightTagRightPixelRightSide = 10; //this is how far to the right the next tag should be from the camera for the robot to drop the right pixel on the right side of the mountain

    public static double distanceBetweenValleys = 3;
    public static double distanceBetweenScoopPositions = 4;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public static double SPEED_GAIN  =  0.01  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN =  0.005 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE= 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN  = 0.15;   //  Clip the turn speed to this max value (adjust for your robot)

// Positions

    public static int randomizerPosition = 1;

    // Positions
    //Single Claw
    public static final double clawOpenPosition = .5;
    public static final double clawClosedPosition = .35;
    //Double Claw

    public static final double leftClawMaxOpenPosition = .8;
    public static final double leftClawMaxClosedPosition = .25;
    public static final double rightClawMaxOpenPosition = .8;
    public static final double rightClawMaxClosedPosition = .25;


    public static double rightClawOpenPosition = .4;
    public static double rightClawClosedPosition = 0;
    public static double leftClawOpenPosition = .7;
    public static double leftClawClosedPosition = .3;



    public static final double launcherSetPosition = 0.0;
    public static final double launcherReleasePosition = 1.0;
    public static final double leftHookUpPosition = 0.5;
    public static final double rightHookUpPosition = 0.5;
    public static final double leftHookDownPosition = 0;
    public static final double rightHookDownPosition = 0;
    public static final double leftHookInPosition = 1.0;
    public static final double rightHookInPosition = 1.0;
    public static final int handAssistRideHeightLevel = 5;
    public static final int handAssistRideHeightDistance = 1;
    public static final boolean handAssistRideHeightAboveLevel = true;
    public static final int handAssistHangHeightLevel = 5;
    public static final int handAssistHangHeightDistance = 1;
    public static final boolean handAssistHangHeightAboveLevel = true;

    public static int sampleDetectionPosition = 2;

// Robot Configuration

    public static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera

    // Adjustments for where home is for the hand
    public static double armRotationDegreesAtHome = -10.6;
    public static double armExtensionInchesAtHome = 0;
    public static int stackDistanceAtHome = 0;
    public static int stackLevelAtHome = 0;
    public static final double armExtensionCollapsedLength = 16.125;
    public static final double armPivotHeight = 11.375;
    public static final double armPivotDistance = 15.65;

    // Adjustments for hand position
    public static final double handHorizontalAdjustment = 0;
    public static final double handVerticalAdjustment = 0;

    //Max Ranges
    public static final double minArmExtensionInches = 0;
    public static final double maxArmExtensionInches = 200/25.4; //200mm stroke
    public static final double minArmDegrees = -12;
    public static final double maxArmDegrees = 170;
    public static final double minWristPosition = 0;
    public static final double maxWristPosition = 0.85;

    // Stack Arrays
    public static final double[] stackHeightOnLevelArray =    {0,.75,1.5,2.25,3 , 5  , 8,11, 14,17,20};
    public static final double[] stackHeightAboveLevelArray = {3,4  ,8  ,12  ,16,18  ,20,22, 23,24,25};
    public static final double[] stackDistanceArray =         {0,0  ,0  ,0   , 0,01.2,3 ,5.2,7 , 8,10};

// Conversions
    public static final double ticksPerRevolution1150 = 145.6;
    public static final double ticksPerRevolution435 = 383.6;

    private static final double gearReductionWheel = 1.0;
    private static final double wheelDiameterInches = 4.0;

    public static final double ticksPerInchWheelDrive = driveEfficiency * (ticksPerRevolution1150 * gearReductionWheel) / (Math.PI * wheelDiameterInches);
    public static final double ticksPerInchWheelStrafe = strafeEfficiency * (ticksPerRevolution1150 * gearReductionWheel) / (Math.PI * wheelDiameterInches);
    public static final double ticksPerDegreeTurnChassis = turnEfficiency * ((ticksPerRevolution1150 * gearReductionWheel) / (Math.PI * wheelDiameterInches))/360;

    private static final double gearReductionArm = 24.0;
    private static final double gearReductionExtensionSpur = 1.0;
    private static final double translationInchPerRotationExtensionScrew = 4 * 2.0 / 25.4; //4 Starts, 2mm pitch, 25.4 mm/inch

    public static final double ticksPerDegreeArm = armRotationEfficiency * (ticksPerRevolution1150 /360 ) * gearReductionArm;
    public static final double ticksPerInchExtension = armExtensionEfficiency * ticksPerRevolution1150 * gearReductionExtensionSpur / translationInchPerRotationExtensionScrew;

    //conversion methods
    public static double armLengthDesired(double horDesired, double vertDesired){
        double horArmLengthDesired = horDesired + armPivotDistance;
        double vertArmLengthDesired = vertDesired - armPivotHeight;
        return Math.sqrt(Math.pow(horArmLengthDesired,2) + Math.pow(vertArmLengthDesired,2));
    }
    public static double wristConversionToServo(double angle){
        /*********************************
         *
         * Returns the wrist servo setting to set the relative 'angle' of the wrist compared to the arm
         *
         ********************************/
        double servoPosition = 0.5;

        double rangeMin = 75;
        double rangeMid1 = 100;
        double rangeMid2 = 180;
        double rangeMid3 = 230;
        double rangeMax = 260;
        double servoMin = 0.17;
        double servoMid1 = 0.261891892;
        double servoMid2 = 0.555945946;
        double servoMid3 = 0.73972973;
        double servoMax = 0.85;

        if(angle < rangeMin){
            servoPosition =  servoMin;
            return servoPosition;
        } else if (angle < rangeMid1){
            servoPosition = prorate(angle, rangeMin, rangeMid1, servoMin, servoMid1);
            return servoPosition;
        } else if (angle < rangeMid2){
            servoPosition = prorate(angle, rangeMid1, rangeMid2, servoMid1, servoMid2);
            return servoPosition;
        } else if (angle < rangeMid3){
            servoPosition = prorate(angle, rangeMid2, rangeMid3, servoMid2, servoMid3);
            return servoPosition;
        } else if (angle < rangeMax){
            servoPosition = prorate(angle, rangeMid3, rangeMax, servoMid3, servoMax);
            return servoPosition;
        } else {
            servoPosition = servoMax;
            return servoPosition;
        }


        /*
        double wristAngles[] = {0, 90.0, 170};
        double wristNumbers[] = {.2, 0.5, .85};
        for(int i=0;i <= wristAngles.length - 1;i++ ){
            if(wristAngles[i]>=angle && i==0){
                servoPosition = wristNumbers[i];
                return servoPosition;

            } else if(wristAngles[i]>=angle && i>0){
                //i is higher: then we use it as the upper bound and prorate down to i-1 angles & numbers
                    servoPosition = prorate(angle,wristAngles[i-1],wristAngles[i],wristNumbers[i-1],wristNumbers[i]);
                    return servoPosition;

            } else {
                    servoPosition = wristNumbers[i];
                    return servoPosition;

            }

        }
        */

    }

    public static double prorate(double givenNumber, double givenRangeLow, double givenRangeHigh, double findRangeLow, double findRangeHigh){
        double findNumber;
        findNumber= findRangeHigh-((givenRangeHigh-givenNumber)/(givenRangeHigh-givenRangeLow)*(findRangeHigh-findRangeLow));
        return findNumber;
    }
    public static int findStackLevel(){
        int level = 0;
        return level;
    }
    public static int findStackDistance(){
        int level = 0;
        return level;
    }


}
