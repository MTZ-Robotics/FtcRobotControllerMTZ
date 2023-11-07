package org.firstinspires.ftc.teamcode;

/***********************************************
 * Constants for use in Autonomous or TeleOp
 *
 * Versions
 * v000 Saved after 08Jan2022 Competition
 * v001 Changes after 08Jan2022 Competition
 * v002 Changes after 22Jan2022 Competition
 * v201 Changes for arm with new shoulder and extension and double claw
 * v202 Corrections on 01Feb2022
 *
 ***********************************************/
public class mtzConstantsPP {
// Adjustments for efficiency
    public static final double driveEfficiency =1.0;
    public static final double strafeEfficiency = 1.0;
    public static final double turnEfficiency = 55.6;
    public static final double armRotationEfficiency = 1.0;
    public static final double armExtensionEfficiency = 1.0;
// Debug Delay
    public static final int defaultPauseTime = 200;  //Milliseconds after a command


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
    public static double defaultArmPower = 0.8;
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
    public static double wristBump = 0.03; // servo rotations

    public static double defaultDriveSpeed = 0.35;
    public static double driveSlowRatio = 0.35;
    public static double driveFastRatio = 1/0.7;

    public static double defaultIntakeSpeed = 0.25;
    public static double defaultFlywheelSpeed = 0.08;


// Positions

    public static int randomizerPosition = 1;

    // Positions
    //Single Claw
    public static final double clawOpenPosition = 0;
    public static final double clawClosedPosition = 1;
    //Double Claw
    public static final double leftClawOpenDuckPosition = .2;
    public static final double leftClawOpenBoxPosition = .15;
    public static final double leftClawOpenBallPosition = .1;
    public static final double leftClawMaxOpenPosition = 0;
    public static double leftClawOpenPosition = leftClawOpenBoxPosition;
    public static final double leftClawDuckPosition = .5;
    public static final double leftClawBoxPosition = .35;
    public static final double leftClawBallPosition = .2;
    public static final double leftClawMaxClosedPosition = 0;
    public static double leftClawClosedPosition = leftClawBoxPosition;
    public static final double rightClawOpenDuckPosition = .2;
    public static final double rightClawOpenBoxPosition = .15;
    public static final double rightClawOpenBallPosition = .1;
    public static final double rightClawMaxOpenPosition = 0;
    public static double rightClawOpenPosition = rightClawOpenBoxPosition;
    public static final double rightClawDuckPosition = .5;
    public static final double rightClawBoxPosition = .35;
    public static final double rightClawBallPosition = .1;
    public static double rightClawClosedPosition = rightClawBoxPosition;
    public static final double rightClawMaxClosedPosition = 0;

    public static final double blockThrowerDownPosition = 0.55;
    public static final double blockThrowerUpPosition = 1.0;
    public static final double leftHookUpPosition = 0.5;
    public static final double rightHookUpPosition = 0.5;
    public static final double leftHookDownPosition = 0;
    public static final double rightHookDownPosition = 0;
    public static final double leftHookInPosition = 1.0;
    public static final double rightHookInPosition = 1.0;
    public static final int handAssistRideHeightLevel = 1;
    public static final int handAssistRideHeightDistance = 1;
    public static final boolean handAssistRideHeightAboveLevel = true;

    public static int sampleDetectionPosition = 2;
    public static int skyStonePosition = 2;


    // Adjustments for where home is for the hand
    public static double armRotationDegreesAtHome = -36.011;
    public static double armExtensionInchesAtHome = 3.22;
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
    public static final double minArmDegrees = -60;
    public static final double maxArmDegrees = 70;
    public static final double minWristPosition = 0.4;
    public static final double maxWristPosition = 1.0;

    // Stack Arrays
    public static final double[] stackHeightOnLevelArray = {0,1,5,9,13};
    public static final double[] stackHeightAboveLevelArray = {3,4,8,12,16};
    public static final double[] stackDistanceArray = {0,1.2,3,5.2,7};

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
    public static final double ticksPerInchExtension = armExtensionEfficiency * ticksPerRevolution435 * gearReductionExtensionSpur / translationInchPerRotationExtensionScrew;

    //conversion methods
    public static double armLengthDesired(double horDesired, double vertDesired){
        double horArmLengthDesired = horDesired + armPivotDistance;
        double vertArmLengthDesired = vertDesired - armPivotHeight;
        return Math.sqrt(Math.pow(horArmLengthDesired,2) + Math.pow(vertArmLengthDesired,2));
    }
    public static double wristConversionToServo(double angle){
        double servoPosition = 0.5;
        double wristAngles[] = {40.00, 90.0, 140};
        double wristNumbers[] = {minWristPosition, 0.69, maxWristPosition};
        for(int i=0;i <= wristAngles.length - 1;i++ ){
            if(wristAngles[i]>=angle && i==0){
                servoPosition = wristNumbers[i];
                break;
            } else
                if(wristAngles[i]>=angle && i>0){
                //i is higher: then we use it as the upper bound and prorate down to i-1 angles & numbers
                    servoPosition = prorate(angle,wristAngles[i-1],wristAngles[i],wristNumbers[i-1],wristNumbers[i]);
                break;
            } else {
                    servoPosition = wristNumbers[i];
                break;
            }

        }
        return servoPosition;
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
