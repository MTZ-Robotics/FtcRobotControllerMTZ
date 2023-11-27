package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armExtensionEfficiency;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armRotationDegreesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.clawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.clawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmLowerPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftHookUpPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightHookUpPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.skyStonePosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeArm;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeTurnChassis;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.wristConversionToServo;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name ="Auto Controls working", group = "z_test")
//Strawberry

//Adds use of Constants File

//@Disabled

public class AutoControlsMTZ_workingCS extends LinearOpMode {


    /**************
     *
     * Modify these speeds to help with diagnosing drive errors
     *
     **************/
    private static final double defaultDriveSpeed = 0.2;
    private static final double defaultTurnSpeed = 0.4;
    private static final int defaultPauseTime = 200;

    /**********************
     * These variables are the constants in path commands
     **********************/
    private static final double ticksPerRevolution = 145.6;
    private static final double gearReduction = 2.0;
    private static final double wheelDiameterInches = 4.0;
    private static final double conversionTicksToInches = (ticksPerRevolution * gearReduction) / (Math.PI * wheelDiameterInches);
    private static final double experimentalInchesPerTurn = 91.8;
    private static final double armDistanceAdjustment = 200.00;
    private static final double strafeDistanceAdjustment = 1.00;
    private static final double driveDistanceAdjustment = 1.00;
    private int allianceReverser = 1;

    double wristPositionDesired = wristConversionToServo(45+armRotationDegreesAtHome);
    double wristAngleDesired = 90;

    /*****************
     * Declare motor & servo objects
     ****************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor arm;
    private DcMotor armExtension;
    private Servo claw;
    //private Servo leftHook;
    //private Servo rightHook;
    //private Servo blockThrower;
    private Servo wrist;
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;


    /***********
     * Lights Control Declarations
     ***********/

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override

    public void runOpMode() throws InterruptedException {
        autoPaths("Red","default",false);

    }

    public void autoPaths(String alliance,String pathToRun,Boolean supportArm) throws InterruptedException {

        boolean oldArm = supportArm;

        /**************
         *
         * Declare motors and servos
         *
         */
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        arm = hardwareMap.dcMotor.get("arm");
        armExtension = hardwareMap.dcMotor.get("armExtension");
        wrist = hardwareMap.servo.get("rightClaw");
        claw = hardwareMap.servo.get("leftClaw");

        //rightHook = hardwareMap.servo.get("rightHook");
        //leftHook = hardwareMap.servo.get("leftHook");
        //blockThrower = hardwareMap.servo.get("blockThrower");
        leftColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color2");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        armExtension.setDirection(DcMotor.Direction.REVERSE);
        //leftHook.setDirection(Servo.Direction.REVERSE);


        /*************
         * Set Lights Variables to the color for the alliance
         *************/
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //Paths written for Blue alliance and reverse turns if on Red alliance

        if (alliance=="Blue") {
            allianceReverser = 1;
            pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        } else if (alliance=="Red") {
            allianceReverser = -1;
            pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        }
        blinkinLedDriver.setPattern(pattern);

        /******
         Movement starts here on initialize
         */
        //leftHook.setPosition(leftHookUpPosition);
        //rightHook.setPosition(rightHookUpPosition);

        StopAndResetAllEncoders();

        /************
         * Raise the arm to fit within 18" long dimension
         ************/
        if (oldArm) {
           RaiseArm(14, defaultArmPower, defaultPauseTime/4,wristAngleDesired);
        }

        telemetry.log().clear();
        telemetry.update();
        telemetry.log().add(pathToRun+" Initialized. Go "+alliance+" alliance");


        /************************************************************
         * Paths            Paths            Paths          Paths   *
         ************************************************************/
        //Write paths for Blue alliance and apply reverser on turns and strafes
        if (pathToRun == "default"){
            pathToRun = "DepotSampleWall";
        }

        if (pathToRun=="FoundationBridge" || pathToRun=="FoundationWall" ) {
            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Robot should face away from wall centered in middle tile foundation side.");
            waitForStart();
            //Turn lights off
            if (alliance=="Blue") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            } else if (alliance=="Red") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
            blinkinLedDriver.setPattern(pattern);
            /************
             * Path Start
             ************/
            //ExtendArm(-5,1,defaultPauseTime);
            //goToFoundationfromWall(allianceReverser);
            //moveFoundation(allianceReverser);
            //Align to Park
            if (pathToRun=="FoundationBridge") {
                foundationSlideToBridge(allianceReverser);
            } else {
                foundationSlideToWall(allianceReverser);
            }
            foundationDriveToPark(allianceReverser);
        } else if (pathToRun=="DepotWall" || pathToRun=="DepotBridge") {
            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Robot starts facing quarry on intersection between tiles one tile away from bridge line.");
            waitForStart();
            //Turn lights off
            if (alliance=="Blue") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            } else if (alliance=="Red") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
            blinkinLedDriver.setPattern(pattern);
            /************
             * Path Start
             ************/
            //Wait 15 seconds
            sleep(15000);
            if (pathToRun=="DepotWall"){
                //Move forward slightly
                Drive(1,defaultDriveSpeed,defaultPauseTime);
            } else {
                //Move forward more
                Drive(12,defaultDriveSpeed,defaultPauseTime);
            }
            //Park
            Strafe(allianceReverser * -24,defaultDriveSpeed,0);
        } else if (pathToRun=="DepotSampleWall" || pathToRun=="DepotSampleBridge") {
            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Line up with wheels away from depot centered on the tile seam towards the last stone of the quarry near bridge facing quarry");
            waitForStart();
            //Turn lights off
            if (alliance=="Blue") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            } else if (alliance=="Red") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
            blinkinLedDriver.setPattern(pattern);
            /************
             * Path Start
             ************/



            double distanceFromStone3ToSkyStone=0;
            /*

            //Move forward to quarry (quarry distance-robot length-distance from stone to measure)
            //Distance 20 works better
            //Drive(48-18-2, .10, 1000);
            sampleSkyStone(allianceReverser);
            //Strafe to SkyStone Position



            if(skyStonePosition==3){
                Strafe(allianceReverser * -4,.1,defaultPauseTime);
                distanceFromStone3ToSkyStone = 0;
            } else if(skyStonePosition==2){
                Strafe(allianceReverser * 4,.1,defaultPauseTime);
                distanceFromStone3ToSkyStone = 8;
            }else if(skyStonePosition==1){
                Strafe(allianceReverser * 12,.1,defaultPauseTime);
                distanceFromStone3ToSkyStone = 16;
            }
             */
            //Raise Arm a little

            wrist.setDirection(Servo.Direction.REVERSE);
            //wrist.setPosition(wristPositionDesired);
            sleep(1000);
            //RaiseArm(20 ,defaultArmPower,defaultPauseTime, wristAngleDesired);

            claw.setPosition(clawClosedPosition);
            /*
            wrist.setPosition(1);
            telemetry.addData("Wrist Position",wrist.getPosition());
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);
            wrist.setPosition(.9);
            telemetry.addData("Wrist Position",wrist.getPosition());
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);
            wrist.setPosition(.8);
            telemetry.addData("Wrist Position",wrist.getPosition());
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);
            wrist.setPosition(.7);
            telemetry.addData("Wrist Position",wrist.getPosition());
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);

             */

/*
            wrist.setPosition(.6);
            telemetry.addData("Wrist Position",wrist.getPosition());
            sleep(1000);
            wrist.setPosition(.5);
            telemetry.addData("Wrist Position",wrist.getPosition());
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);
            wrist.setPosition(.4);
            telemetry.addData("Wrist Position",wrist.getPosition());

 */
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);
            wrist.setPosition(.3);
            telemetry.addData("Wrist Position",wrist.getPosition());
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);
            wrist.setPosition(.2);
            telemetry.addData("Wrist Position",wrist.getPosition());
            //Get claw & wrist off of arm and ready to grab
            sleep(1000);
            wrist.setPosition(.1);
            telemetry.addData("Wrist Position",wrist.getPosition());
            sleep(1000);
            wrist.setPosition(0);
            telemetry.addData("Wrist Position",wrist.getPosition());
            /*
            //Extend Arm
            //ExtendArm(5,1,defaultPauseTime);
            Drive(1,.1,defaultPauseTime);
            //grabSkyStone(allianceReverser);
            //Reverse to clear bridge post
            Drive(-12,defaultDriveSpeed/2,defaultPauseTime);
            //Strafe to foundation with block and add on any distance past stone 3 location
            Strafe((allianceReverser*-3*24)+8 + (int)distanceFromStone3ToSkyStone,defaultDriveSpeed,defaultPauseTime);
            //Forward since we are past bridge post
            Drive(8,defaultDriveSpeed/2,defaultPauseTime);
            //Open claw to allow block to be pushed into foundation

             */
            claw.setPosition(clawOpenPosition);
            Drive(4,defaultDriveSpeed/4,defaultPauseTime);
            //Lower Arm a little
            //RaiseArm(-20,defaultArmLowerPower,defaultPauseTime,wristAngleDesired);

            /*
            moveFoundation(allianceReverser);
            //Once arm is working, we can use this here
            //claw.setPosition(clawOpenPosition);
            //Align to Park
            if (pathToRun=="DepotSampleBridge") {
                foundationSlideToBridge(allianceReverser);
            } else {
                foundationSlideToWall(allianceReverser);
            }
            foundationDriveToPark(allianceReverser);
            //Retract Arm
            ExtendArm(-5,1,0);


             */
        } else if (pathToRun=="FoundationSampleWall") {
            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Robot should face away from wall centered in middle tile foundation side.");
            waitForStart();
            //Turn lights off
            if (alliance=="Blue") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            } else if (alliance=="Red") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
            blinkinLedDriver.setPattern(pattern);
            /******************
             * Path Pseudo Code
             ******************
             * Start facing wall, center of foundation w/ arm raised
             * Go to Foundation
                * Backwards fast 30
                * Backwards slow for 5
             * Move Foundation
                * Hooks Down
                * Turn Wall to Audience with forward 90°
                * Backwards 24 Fast
                * Hooks Up
             * Travel to Audience
                * Forwards 10
                * Turn Audience to Bridge 90° Fast
                * Backwards 24
                * Strafe towards audience Fast with tweak towards wall
                * Strafe towards audience slow for 6
             * Sample
                * Turn on Vuforia
                * Backwards 10 medium
                * Strafe towards Building Site 6
                * Sample Stone (Ends with arm raised claw closed)
             * Go to foundation
             * Turn bridge to Building Site 90° Fast
             * Strafe towards Wall 24 Fast
             * Forward with tweak towards Wall for 96
             * Strafe Bridge and forward 6
             * Open claw
             * Strafe Wall and backwards 40
             */

            /************
             * Path Start
             ************/
            goToFoundationfromWall(allianceReverser);
            moveFoundation(allianceReverser);
            foundationToAudienceDepot(allianceReverser);
            sampleSkyStone(allianceReverser);
            grabSkyStone(allianceReverser);
            quarryToMovedFoundation(allianceReverser);
            //Open Claw
            claw.setPosition(0);
            sleep(500);
            //Back up to park against wall
            Drive(-30, defaultDriveSpeed*2, defaultPauseTime);
        } else if (pathToRun=="Calibrate") {
            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Robot Raises Arm 10, Moves Forward 24, then Left 24, then Rotate Left 180");
            waitForStart();
            /************
             * Path Start
             ************/
            RaiseArm(10,defaultArmPower, 2000,wristAngleDesired);
            Drive(24,defaultDriveSpeed,5000);
            Strafe(24,defaultDriveSpeed,5000);
            Turn(-180,defaultTurnSpeed,0);
        } else {
            /************************************
             * Path Selection Error
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Error in Path Selection");
            telemetry.update();
            if (alliance=="Blue") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE;
            } else if (alliance=="Red") {
                pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
            }
            blinkinLedDriver.setPattern(pattern);
            waitForStart();
            /************
             * Path Start
             ************/
            sleep(30000);
        }

        // End of Paths
    }

    /**********************
     * Path Methods
     **********************/
    public void goToFoundationfromWall(int allianceReverser) throws InterruptedException{

        //Align Hooks With Foundation
        Drive(24, defaultDriveSpeed, defaultPauseTime);
        Strafe(allianceReverser * -12, defaultDriveSpeed, defaultPauseTime);
        Drive(5, defaultDriveSpeed, defaultPauseTime);
    }

    public void moveFoundation (int allianceReverser) throws InterruptedException{

        //Hook Foundation
        HooksDown();

        //Move Foundation to Build Zone
        Drive(-20, 0.2, defaultPauseTime);
        Turn(allianceReverser * -40, 0.2, defaultPauseTime);
        Drive(-5, 0.2, defaultPauseTime);
        Turn(allianceReverser * -80, 0.2, defaultPauseTime);
        Strafe(allianceReverser * 5, 0.2, defaultPauseTime);
        Drive(12, 0.1, defaultPauseTime);

        //Unhook Foundation
        HooksUp();
    }

    public void foundationToAudienceDepot(int allianceReverser) throws InterruptedException {
        /***
        * Travel to Audience
        * Forwards 10
        * Turn Audience to Bridge 90° Fast
        * Backwards 24
        * Strafe towards audience Fast with tweak towards wall
        * Strafe towards audience slow for 6
        */

        Drive(18, defaultDriveSpeed, defaultPauseTime);
        Turn(allianceReverser * -90, defaultTurnSpeed, defaultPauseTime);
        Drive(-30, defaultDriveSpeed, defaultPauseTime);
        Strafe(allianceReverser * -4*24, defaultDriveSpeed*2, defaultPauseTime);
        Drive(-12, defaultDriveSpeed, defaultPauseTime);
        Drive(10, defaultDriveSpeed, defaultPauseTime);
        Strafe(allianceReverser * -18, defaultDriveSpeed/2, defaultPauseTime);
        Strafe(allianceReverser * 8, defaultDriveSpeed/2, defaultPauseTime);

    }

    public void quarryToMovedFoundation (int allianceReverser) throws InterruptedException{
        /*********
         * Turn bridge to Building Site 90° Fast
         * Strafe towards Wall 24 Fast
         * Forward with tweak towards Wall for 96
         * Strafe Bridge and forward 6
         */
        Turn(allianceReverser*-90,defaultTurnSpeed,defaultPauseTime);
        Strafe(allianceReverser*24,defaultDriveSpeed,defaultPauseTime);
        Drive(3*24, defaultDriveSpeed*2, defaultPauseTime);
        Turn(allianceReverser*20,defaultTurnSpeed,defaultPauseTime);
        Drive(24, defaultDriveSpeed*2, defaultPauseTime);
    }
    public void foundationSlideToBridge(int aR) throws InterruptedException{
        Strafe(aR * 6, defaultDriveSpeed, defaultPauseTime);
    }
    public void foundationSlideToWall(int aR) throws InterruptedException{
        Strafe(aR * -12, defaultDriveSpeed, defaultPauseTime);
    }
    public void foundationDriveToPark(int aR) throws InterruptedException{
        //Forward to bridge area
        Drive(-22, defaultDriveSpeed, defaultPauseTime);//Used to be 24
        //Park
        Drive(-14, defaultDriveSpeed/2, 0);
    }
    public void grabSkyStone(int allianceReverser) throws InterruptedException {
        //CloseClaw();
        claw.setPosition(clawClosedPosition);
        //Wait for it to close
        sleep(1000);
        //Raise Arm
        RaiseArm(40, defaultArmPower, defaultPauseTime,wristAngleDesired);
    }

    //Sampling Methods

    public void sampleSkyStone (int allianceReverser) throws InterruptedException {
        //Determine which of the 3 positions to go after
        //position 1 = audience side (Red)
        //position 2 = middle        (Yellow)
        //position 3 = bridge side   (Green)
        // if blue alliance and location = right, then position 2
        // if Red alliance and location = right, then position 3

        int skyStoneLocation = determineSkyStoneColorSensor();

        skyStonePosition = 3;
        pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
        if (skyStoneLocation == 1) {
            if (allianceReverser == 1) {
                skyStonePosition = 3;
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else {
                skyStonePosition = 2;
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            }

        } else if (skyStoneLocation == 2) {
            if (allianceReverser == 1) {
                skyStonePosition = 2;
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            } else {
                skyStonePosition = 3;
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            }

        } else if (skyStoneLocation == 0) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            skyStonePosition = 1;
        }
        blinkinLedDriver.setPattern(pattern);

    }
    public int determineSkyStoneColorSensor() {
        float lefthsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV(leftColorSensor.red() * 8, leftColorSensor.green() * 8, leftColorSensor.blue() * 8, lefthsvValues);
        float righthsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV(rightColorSensor.red() * 8, rightColorSensor.green() * 8, rightColorSensor.blue() * 8, righthsvValues);

        // Left = 1
        // Right = 2
        // Not Seen = 0
        int skyStonePos = 0;

        //Need to measure this and adjust the constant,
        //Color sensors are different versions; Left is V2 & Right is V3
        // L 60, R 90 are values that worked in lower light
        if(lefthsvValues[0]>60){
            skyStonePos = 1;
        } else if(righthsvValues[0]>90){
            skyStonePos = 2;
        }

        return skyStonePos;
    }
    //Motion Methods

    public void Drive(int distance, double motorPower, int pause) throws InterruptedException {
       if (opModeIsActive()) {
            StopAndResetDriveEncoders();
            DriveByInches(distance);
            RunDriveToPosition();
            DrivePower(motorPower);
            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                DisplayDriveTelemetry();
            }
            DrivePower(0);
            Thread.sleep(pause);
        }
    }
    public void Strafe(int leftDistance, double power, int pause) throws InterruptedException {
        //Left is positive
        if (opModeIsActive()) {
            StopAndResetDriveEncoders();
            StrafeByInches(leftDistance);
            RunDriveToPosition();
            DrivePower(power);
            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                DisplayDriveTelemetry();
            }
            DrivePower(0);
            Thread.sleep(pause);
        }
    }
    public void Turn(int rightDegrees, double power, int pause) throws InterruptedException {
        //Left is negative
        if (opModeIsActive()) {
            StopAndResetDriveEncoders();
            TurnByAngle(rightDegrees);
            RunDriveToPosition();
            DrivePower(power);
            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                DisplayDriveTelemetry();
            }
            DrivePower(0);
            Thread.sleep(pause);
        }
    }
    public void RaiseArmByDegrees(double degrees, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            RunArmToPosition();
            RaiseByInches (degrees);
            ArmPower(defaultArmPower);
        }
        ArmPower(0);
        Thread.sleep(pause);

    }

    public void LowerArm(int distance, int pause) throws InterruptedException {
        /*arm.setPower(0.1);
        sleep(distance * 100);
        arm.setPower(0);
        Thread.sleep(pause);

         */
    }

   /* public void RaiseArm(double degrees, double power,int pause) throws InterruptedException {
            if (opModeIsActive()) {
                raiseByDegrees(degrees);
                ArmPower(power);
                while (arm.isBusy() || armExtension.isBusy()) {
                    DisplayArmTelemetry();
                }
            }
            Thread.sleep(pause);

    }*/
    public void RaiseArm(double degrees, double power,int pause, double wristAngle) throws InterruptedException {
       // if(hasArmMotorsAndServos) {
            if (opModeIsActive()) {
                raiseByDegrees(degrees);
                RunArmToPosition();
                ArmPower(power);
                while (arm.isBusy() || armExtension.isBusy()) {
                    DisplayArmTelemetry();
                }
            }
            wrist.setPosition(wristConversionToServo(wristAngle + (armRotationDegreesAtHome + arm.getCurrentPosition() / ticksPerDegreeArm)));
            Thread.sleep(pause);
       // }
    }
    /*public void RaiseArm(int distance, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            RaiseByInches (distance);
            ArmPower(defaultArmPower);
        }
        while(arm.isBusy()){
            DisplayArmTelemetry();
        }
        ArmPower(0);
        Thread.sleep(pause);

    }*/
    public void ExtendArm(double additionalExtension, double power,int pause) throws InterruptedException {
        if (opModeIsActive()) {
            armExtension.setTargetPosition((int) ((armExtension.getCurrentPosition() + additionalExtension) * ticksPerInchExtension));
            RunArmExtensionToPosition();
            armExtension.setPower(power);
            while (arm.isBusy() || armExtension.isBusy()) {
                DisplayArmTelemetry();
            }
            armExtension.setPower(0);
        }
        Thread.sleep(pause);
    }
    public void HooksDown()throws InterruptedException {
        //Light Reverse Power On
        lightForward();
        sleep(500);

        //leftHook.setPosition(0);
        //rightHook.setPosition(0);
        sleep(1500);

        //Reverse Power Off
        StopAndResetDriveEncoders();
    }
    public void HooksUp() {
        //leftHook.setPosition(0.5);
        //rightHook.setPosition(0.5);
        sleep(1500);
    }
    public void lightForward() throws InterruptedException{

        //This is not making all of the wheels turn in the same direction and so it is commented out
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //DrivePower(-0.1);

        //Substituting this instead
        Drive(1,0.1,defaultPauseTime);
    }

    public void DisplayArmTelemetry() {
        double armInches = arm.getCurrentPosition() / conversionTicksToInches;
        telemetry.clear();
        telemetry.addLine()
                .addData("Arm Inches ",  armInches + "  Power: " + "%.1f", arm.getPower())
                .addData("Arm Extension Inches ", armExtension.getCurrentPosition() / armExtensionEfficiency + "  Power: " + "%.1f", armExtension.getPower());
        telemetry.update();
    }

//Encoder Methods

    public void StopAndResetAllEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void StopAndResetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void StopAndResetArmEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void RunDriveToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void RunArmToPosition() {

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void RunArmExtensionToPosition() {
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //End of Encoder Methods

//Distance Calculation Methods

    public void DriveByInches(int distance) {
        double correctedDistance = (distance*(strafeDistanceAdjustment));
        frontLeft.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        frontRight.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        backLeft.setTargetPosition((int)(-1 * correctedDistance * conversionTicksToInches));
        backRight.setTargetPosition((int)(-1 * correctedDistance * conversionTicksToInches));
    }

    public void StrafeByInches(int distance) {
        double correctedDistance = distance*(strafeDistanceAdjustment);
        frontLeft.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        frontRight.setTargetPosition((int)(-correctedDistance * conversionTicksToInches));
        backLeft.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        backRight.setTargetPosition((int)(-correctedDistance * conversionTicksToInches));
    }

    public void TurnByAngle(int degrees) {
        frontLeft.setTargetPosition((int)(degrees * ticksPerDegreeTurnChassis));
        frontRight.setTargetPosition((int)(-degrees * ticksPerDegreeTurnChassis));
        backLeft.setTargetPosition((int)(-degrees * ticksPerDegreeTurnChassis));
        backRight.setTargetPosition((int)(degrees * ticksPerDegreeTurnChassis));
    }
    public void RaiseByInches(double distance) {
        int correctedDistance = (int) (distance * (armDistanceAdjustment));
        arm.setTargetPosition(correctedDistance);
    }

    public void raiseByDegrees(double degrees) {
            arm.setTargetPosition((int)((degrees - armRotationDegreesAtHome) * ticksPerDegreeArm));
    }

    //End of distance calculation methods

//Power Methods

    public void DrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void ArmPower(double power) {
        arm.setPower(power);
    }
//End Power Methods

    //Telemetry Methods

    public void DisplayDriveTelemetry() {
        double frontLeftInches = frontLeft.getCurrentPosition() / conversionTicksToInches;
        double frontRightInches = frontRight.getCurrentPosition() / conversionTicksToInches;
        double backLeftInches = backLeft.getCurrentPosition() / conversionTicksToInches;
        double backRightInches = backRight.getCurrentPosition() / conversionTicksToInches;
        telemetry.clear();
        telemetry.addLine()
                .addData("Front Left Inches ", (int) frontLeftInches + "   Power: " + "%.1f", frontLeft.getPower());
        telemetry.addLine()
                .addData("Front Right Inches: ", (int) frontRightInches + "   Power: " + "%.1f", frontRight.getPower());
        telemetry.addLine()
                .addData("Back Left Inches: ", (int) backLeftInches + "   Power: " + "%.1f", backLeft.getPower());
        telemetry.addLine()
                .addData("Back Right Inches: ", (int) backRightInches + "   Power: " + "%.1f", backRight.getPower());
        telemetry.update();
    }
    //End of Telemetry Methods
    //End of Class
}
