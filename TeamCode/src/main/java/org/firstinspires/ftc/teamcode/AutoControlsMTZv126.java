package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.armRotationDegreesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.cameraBearingOffsetLeftTagLeftPixelLeftSide;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.cameraBearingOffsetRightTagRightPixelRightSide;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.distanceBetweenScoopPositions;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.distanceBetweenValleys;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.leftClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.leftClawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.randomizerPosition;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.rightClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.rightClawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.ticksPerDegreeArm;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.ticksPerDegreeTurnChassis;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.ticksPerInchWheelDrive;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.ticksPerInchWheelStrafe;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.ticksPerRevolution1150;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.wristConversionToServo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name ="Auto Controls v126", group = "Bottom")
@Disabled

/*************************
 * This class is intended to be a sub class to run the robot autonomously.
 * The following methods are available to super classes:
 *
 *      autoPaths  specify different path strings to use and whether the arm needs support
 *
 * v100 Copied from Last Year
 * v101 Updated paths from Meet 1 & added logging
 * v102 Cleaned Up Code
 * v103 Lucy worked on path for drop and parking at practice
 * v104 Fine tuned paths and added right side to Shannon & Lucy
 * v105 Competition Updates during meet 2
 * v106 Adding in Gyro
 * v107 Correcting Gyro at 30Jan2023 Practice
 * v108 From 2023
 * v109 Added paths for Center Stage without sensing
 * v110 Last update before test code added
 * v111 Added simpler paths and parking areas
 * v112 Updates during Meet 1
 * v113 Updates before Thanksgiving
 * v114 Added AutoAlign
 * v115 Prior to Adding Paths
 * v116
 * v117 Added Dropping at backdrop and looking for tag on other alliance and camera looks at left or right tag
 * v118 Removed Old Paths & added turn towards backdrop from backdrop side
 * v119 Added Early Delay Option
 * v120 Changes made at Meet 2
 * v121 Changes made during 12/14 practice
 * v122 Changes after 12/14 practice
 * v123 Changes made during 12Jan2024
 * v124 Changes made during 12Jan2024
 * v125
 * v126 Changes made during Meet 3
 *
 *
 *
 *******************/

public class AutoControlsMTZv126 extends LinearOpMode {


    /**************
     *
     * Modify these speeds to help with diagnosing drive errors
     *
     **************/
    private static final double defaultDriveSpeed = 0.2;
    private static final double defaultTurnSpeed = 0.2;
    private static int defaultPauseTime = 10;

    /**********************
     * These variables are the constants in path commands
     **********************/
    private static final double ticksPerRevolution = ticksPerRevolution1150;
    private static final double gearReduction = 1.0;
    private static final double wheelDiameterInches = 4.0;

    private static final double pi = 3.1415;
    private static final double conversionTicksToInches = (ticksPerRevolution * gearReduction) / (pi * wheelDiameterInches);
    private static final double armDistanceAdjustment = 39.4;

    private static final double defaultArmPower = 0.3;

    //private static final double strafeDistanceAdjustment = 1.15;

    //private static final double driveDistanceAdjustment = .85;
    private int allianceReverser = 1;


    public int armOdometer=0;
    public int extendOdometer=0;


    /******************
     * April Tag Alignment Declarations
     */

    private static int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    /******************
     * Declare the gyro
     */


    // The IMU sensor object
    BNO055IMU imuForDisplay;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private double currAngle = 0.0;


    /*****************
     * Declare motor & servo objects
     ****************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor arm;
    private DcMotor armExtension;
    private Servo wrist;
    private Servo leftClaw;
    private Servo rightClaw;
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;


    /**************
     * Sampling variables
     */

    int randomNumberResult = 2;

    /***********
     * Lights Control Declarations
     ***********/

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    /*********************
     * Start TensorFlow Set-up
     */

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;


    //End TensorFlow Set-up
    //Start AprilTag

    @Override

    /**************
     *
     * The method below runs when this specific opMode is selected to run.
     * Typically this is used for testing versions of this opMode
     * since the super classes will only call the unversioned file
     *
     * A typical path is used for reference between versions
     *
     **************/
    public void runOpMode() throws InterruptedException {
        try {
            autoPaths("Red","default",false);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    /**************
     *
     * The autoPaths method is called from super classes to specify the alliance and path
     *
     **************/
    public void autoPaths(String alliance,String pathToRun,Boolean supportArm) throws InterruptedException, IOException {

        Logging.setup();
        Logging.log("Starting AutoPaths method");
        Logging.log("Path to Run: " + pathToRun);

        /************
         * Assign gyro
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imuForDisplay = hardwareMap.get(BNO055IMU.class, "imu");
        imuForDisplay.initialize(parameters);


        /**************
         *
         * Assign motors and servos
         *
         */
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color2");

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw.setDirection(Servo.Direction.REVERSE);
        //rightClaw.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");
        arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setDirection(DcMotor.Direction.REVERSE);


        armExtension = hardwareMap.dcMotor.get("armExtension");
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setDirection(DcMotor.Direction.REVERSE);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armOdometer=0;



        //Sampling Variables
        randomizerPosition = 2;


        /*************
         * Set Lights Variables to the color for the alliance
         *************/
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        if (alliance=="Blue") {
            pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        } else if (alliance=="Red") {
            pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        }
        blinkinLedDriver.setPattern(pattern);


        /********
         * Movement starts here on initialize
         */


        //This was commented out in the code that was running in meet 1
        //Leaving it in to see if the arm behaves better after being reset
        StopAndResetAllEncoders();

        leftClaw.setPosition(leftClawClosedPosition);
        rightClaw.setPosition(rightClawClosedPosition);
        //}
        telemetry.log().clear();
        telemetry.update();
        telemetry.log().add(pathToRun+" Initialized. Go "+alliance+" alliance");

        //Paths written for Blue alliance and reverse turns if on Red alliance
        allianceReverser=1;
        if (alliance=="Blue") {
            allianceReverser=-1;
        }


        /************************************************************
         * ******************************************************** *
         * ******************************************************** *
         *
         * Paths            Paths            Paths          Paths   *
         *
         * ******************************************************** *
         * ******************************************************** *
         ************************************************************/




        /*******************
         * Default Path
         ******************/

        if (pathToRun=="default"){
            pathToRun="Backdrop Align";
        }

        double distanceBetweenStartingPositions = 0;

        //if (pathToRun == "Audience" || pathToRun == "Audience Wall" || pathToRun == "Audience Sample" || pathToRun == "Audience Align"){distanceBetweenStartingPositions = 54;}
        //Replaced the code above to try to simplify code
        if(pathToRun.contains("Audience")){
            distanceBetweenStartingPositions = 54;
        }
        /*****************************************************************************
         * Coding Instructions
         *
         * Write paths for Blue alliance and apply reverser on turns and strafes
         *
         ****************************************************************************/


        if(pathToRun.contains("Test")){

            /******************************************************************
             *                           Path Branch Align
             *****************************************************************/

            Logging.log("Running Path Branch Align");
            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Setup facing spike marks where robot can see the left spike. ");

            //Copied from our auto Align class
            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
            double  drive           = 0;        // Desired forward power/speed (-1 to +1)
            double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
            double  turn            = 0;        // Desired turning power/speed (-1 to +1)

            // Initialize the Apriltag Detection process
            initAprilTag();


            if (USE_WEBCAM)
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

            // Wait for driver to press start


            //End Copy


            waitForStart();



            RaiseArmByDegrees(20,defaultPauseTime);
            wrist.setPosition(wristConversionToServo(180));
            if(pathToRun.contains("Early Delay")){
                sleep(mtzConstants_ItD.earlyDelayPauseTime);
            }

            colorSensePixelLocation(allianceReverser);

            //dump pixel

            if(randomizerPosition == 1) {
                Turn(-45, defaultDriveSpeed, defaultPauseTime);
            } else if (randomizerPosition == 3) {
                Turn(45, defaultDriveSpeed, defaultPauseTime);
            }

            //Drive(-10,defaultDriveSpeed,defaultPauseTime);
            /*******************************
             *
             * Check Here
             * 
             * Check all Raise arm negatives
             * Raise Arm by degrees probably goes to a global degrees, not relative
             */
            RaiseArmByDegrees(-4,defaultPauseTime);
            Drive(2,defaultDriveSpeed,defaultPauseTime);
            wrist.setPosition(wristConversionToServo(120));
            rightClaw.setPosition(rightClawOpenPosition);
            Drive(4,defaultDriveSpeed,defaultPauseTime);
            wrist.setPosition(wristConversionToServo(180));
            Drive(-8,defaultDriveSpeed,defaultPauseTime);

            if(pathToRun.contains("Audience")) { //drive through the wall side of the rigging
                if (randomizerPosition == 1) {
                    Turn(45, defaultDriveSpeed, defaultPauseTime);
                } else if (randomizerPosition == 3) {
                    Turn(-45, defaultDriveSpeed, defaultPauseTime);
                }


                Turn(90 * allianceReverser, defaultTurnSpeed, defaultPauseTime); //Turn towards the backdrop
                Drive(-8,defaultDriveSpeed,defaultPauseTime);//avoid the rigging with the scoop
                Strafe(40 * allianceReverser, defaultDriveSpeed, defaultPauseTime); //head towards wall
                //Strafe(3 * allianceReverser, defaultDriveSpeed / 2, defaultPauseTime); //straighten up on the wall


                Strafe(-4 * allianceReverser, defaultDriveSpeed, defaultPauseTime); //move away from the wall
                Drive(8 + 24 + distanceBetweenStartingPositions, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area
                Strafe(-22 * allianceReverser, defaultDriveSpeed, defaultPauseTime); //slide in front of backdrop. Strafe isn't working so we just took this one out.
                Turn(5 * allianceReverser, defaultDriveSpeed, defaultPauseTime);

                /*Road Runner path to test for Red Backdrop from wall to left spike and go to backdrop and pick up from white pixle stack
                TrajectorySequence Audience Red RR = drive.trajectorySequenceBuilder(new Pose2d(-37.52, -67.59, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.89, -32.06), Math.toRadians(90.00))
                .splineTo(new Vector2d(-46.14, -31.43), Math.toRadians(176.10))
                .splineTo(new Vector2d(55.39, -35.42), Math.toRadians(-1.92))
                .setReversed(true)
                .splineTo(new Vector2d(35.00, -57.28), Math.toRadians(223.08))
                .splineTo(new Vector2d(32.69, -58.34), Math.toRadians(0.00))
                .splineTo(new Vector2d(-52.45, -56.44), Math.toRadians(178.73))
                .setReversed(false)
                .splineTo(new Vector2d(-67.38, -35.84), Math.toRadians(125.92))
                .splineTo(new Vector2d(-51.19, -58.34), Math.toRadians(270.00))
                .splineTo(new Vector2d(-51.19, -60.65), Math.toRadians(8.88))
                .splineTo(new Vector2d(29.54, -61.49), Math.toRadians(-0.60))
                .setReversed(true)
                .splineTo(new Vector2d(58.13, -38.15), Math.toRadians(40.16))
                .setReversed(false)
                .splineTo(new Vector2d(56.44, -39.84), Math.toRadians(-10.30))
                    .build()*/

                /*Road Runner path to test for Blue Backdrop from wall to left spike and go to backdrop and pick up from white pixle stack
                TrajectorySequence Audience Red RR = drive.trajectorySequenceBuilder(new Pose2d(-37.52, 67.59, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-36.89, 32.06), Math.toRadians(270.00))
                .splineTo(new Vector2d(-46.14, 31.43), Math.toRadians(183.90))
                .splineTo(new Vector2d(55.39, 35.42), Math.toRadians(361.92))
                .setReversed(true)
                .splineTo(new Vector2d(35.00, 57.28), Math.toRadians(136.92))
                .splineTo(new Vector2d(32.69, 58.34), Math.toRadians(360.00))
                .splineTo(new Vector2d(-52.45, 56.44), Math.toRadians(181.27))
                .setReversed(false)
                .splineTo(new Vector2d(-67.38, 35.84), Math.toRadians(234.08))
                .splineTo(new Vector2d(-51.19, 58.34), Math.toRadians(90.00))
                .splineTo(new Vector2d(-51.19, 60.65), Math.toRadians(351.12))
                .splineTo(new Vector2d(29.54, 61.49), Math.toRadians(360.60))
                .setReversed(true)
                .splineTo(new Vector2d(58.13, 38.15), Math.toRadians(319.84))
                .setReversed(false)
                .splineTo(new Vector2d(56.44, 39.84), Math.toRadians(370.30))
                .build();*/


                /*Road Runner path for Red backdrop starting from middle of spikes and go to backdrop and pick up from white pixle stack
                TrajectorySequence Red backdrop without color sense = drive.trajectorySequenceBuilder(new Pose2d(11.04, -34.58, Math.toRadians(90.00)))
                .splineTo(new Vector2d(56.86, -34.79), Math.toRadians(-0.26))
                .setReversed(true)
                .splineTo(new Vector2d(32.06, -59.60), Math.toRadians(225.00))
                .setReversed(false)
                .splineTo(new Vector2d(-42.57, -57.71), Math.toRadians(178.55))
                .splineTo(new Vector2d(-65.90, -35.00), Math.toRadians(135.78))
                .splineTo(new Vector2d(-55.18, -54.55), Math.toRadians(-61.26))
                .splineTo(new Vector2d(20.71, -58.13), Math.toRadians(-2.70))
                .splineTo(new Vector2d(57.92, -37.94), Math.toRadians(28.47))
                .setReversed(true)
                .build();*/

                /*Road Runner path for Red backdrop starting from middle of spikes and go to backdrop and pick up from white pixle stack
                TrajectorySequence Blue Backdrop without color sense = drive.trajectorySequenceBuilder(new Pose2d(9.78, 32.06, Math.toRadians(270.00)))
                .splineTo(new Vector2d(56.02, 36.05), Math.toRadians(362.89))
                .setReversed(true)
                .splineTo(new Vector2d(28.06, 66.11), Math.toRadians(132.92))
                .setReversed(false)
                .splineTo(new Vector2d(-38.36, 59.81), Math.toRadians(180.00))
                .splineTo(new Vector2d(-54.34, 35.00), Math.toRadians(237.22))
                .splineTo(new Vector2d(-65.06, 36.26), Math.toRadians(175.76))
                .splineTo(new Vector2d(-58.34, 47.40), Math.toRadians(418.88))
                .splineTo(new Vector2d(37.10, 60.86), Math.toRadians(368.02))
                .splineTo(new Vector2d(32.27, 39.84), Math.toRadians(257.05))
                .splineTo(new Vector2d(57.71, 34.37), Math.toRadians(3.95))
                .build();*/


            } else {/*
                if(allianceReverser>0) { //Red
                    if (randomizerPosition == 1) {
                        Turn(-180, defaultDriveSpeed, defaultPauseTime);
                    } else if (randomizerPosition == 2) {
                        Turn(90, defaultDriveSpeed, defaultPauseTime);
                    }
                } else { //Blue
                    if (randomizerPosition == 3) {
                        Turn(180, defaultDriveSpeed, defaultPauseTime);
                    } else if (randomizerPosition == 2) {
                        Turn(-90, defaultDriveSpeed, defaultPauseTime);
                    }
                }
                if(!pathToRun.contains("Align")){
                    Drive(24, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area
                }
            }


            //align to the april tag for the drop

            RaiseArmByDegrees(60,defaultPauseTime);

            if(pathToRun.contains("Align")) {

                /*************************
                 * Align with Camera
                 */
            /*
                int backdropTag = randomizerPosition;
                if (allianceReverser > 0) {
                    backdropTag = randomizerPosition + 3;
                }
                RunDriveWithOutEncoders();
                alignToAprilTag(backdropTag, randomizerPosition == 3, true, true);
            } else {
                /*************************
                 * Old Align by Distance
                 */
                //Old Align by distance
                /*
                Strafe(-2,defaultDriveSpeed,defaultPauseTime);
                if(randomizerPosition==1){
                    Strafe(-6,defaultDriveSpeed,defaultPauseTime);
                } else if(randomizerPosition==3){
                    Strafe(6,defaultDriveSpeed,defaultPauseTime);
                }
                Drive(8,defaultDriveSpeed,defaultPauseTime);
            }


            //Drop Tag Off

            RaiseArmByDegrees(40,defaultPauseTime);
            wrist.setPosition(wristConversionToServo(140-(armOdometer/ticksPerDegreeArm)+armRotationDegreesAtHome));
            ExtendArm(4,defaultArmExtensionPower,defaultPauseTime);
            leftClaw.setPosition(leftClawOpenPosition);
            rightClaw.setPosition(rightClawMaxOpenPosition);
            //wrist.setPosition(wristConversionToServo(190-(armOdometer/ticksPerDegreeArm)+armRotationDegreesAtHome));
            RaiseArmByDegrees(10,defaultPauseTime);
            Drive(-4,defaultDriveSpeed,defaultPauseTime);

            //Park

            park(pathToRun.contains("Park Wall"),pathToRun.contains("Spin"));



            //End Copy

            */
        }
        /*else if (pathToRun.contains("Arm") && pathToRun.contains("Test")) {

                /******************************************************************
                 *                           Path Branch Arm Test
                 *****************************************************************/

                /*Logging.log("Running Path Branch Arm Test");
                /************************************
                 * Path set up -- Add to each path
                 ***********************************/
                //Robot Setup Notes
                /*telemetry.log().add("Line up notes should be entered in. ");
                waitForStart();

            //arm.setPower();

            RaiseArmByDegrees(10, defaultPauseTime);
            sleep(20000);
            RaiseArmByDegrees(-10, defaultPauseTime);
            */

        }

        else if (pathToRun.contains("Calibrate")) {

            /******************************************************************
             *                           Path Branch Calibrate
             *****************************************************************/

            Logging.log("Running Path Branch Calibrate");

            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Robot Raises Arm 10, Moves Forward 24, then Left 24, then Rotate Left 180");
            waitForStart();
            /************
             * Path Start
             ************/
            RaiseArmByDegrees(10,2000);
            Drive(24,defaultDriveSpeed/3,5000);
            Strafe(-24,defaultDriveSpeed/2,5000);
            Turn(-180,defaultTurnSpeed,0);
            ExtendArm(10, defaultArmExtensionPower,2000);
            sleep(2000);
            RaiseArmByDegrees(90,2000);
            sleep(2000);
            ExtendArm(-10, defaultArmExtensionPower,2000);
            RaiseArmByDegrees(armRotationDegreesAtHome,2000);
            /************
             * Path End *
             ***********/
        }

        else if (pathToRun.contains("auto2024")) {

            /******************************************************************
             *                           Path Branch 2024
             *****************************************************************/

            Logging.log("Running Path Auto 2024");

            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Left side lines up center of field");
            waitForStart();
            /************
             * Path Start
             ************/

            RaiseArmByDegrees(60,1000);
            Drive(17.5,defaultDriveSpeed,1000);
            RaiseArmByDegrees(-2,1000);
            rightClaw.setPosition(rightClawOpenPosition);
            leftClaw.setPosition(leftClawOpenPosition);
            Drive(-6,defaultDriveSpeed,1000);
            RaiseArmByDegrees(98,1000);
            Strafe(24,defaultDriveSpeed,1000);
            Drive(34,defaultDriveSpeed,1000);
            Turn(89,defaultTurnSpeed,1000);
            Drive(6.5,defaultDriveSpeed,1000);
            Strafe(46,defaultDriveSpeed,1000);
            Strafe(-46,defaultDriveSpeed,1000);
            Drive(6.5,defaultDriveSpeed,1000);
            Strafe(46,defaultDriveSpeed,1000);
            Strafe(-48,defaultDriveSpeed,1000);
            Drive(7,defaultDriveSpeed,1000);
            Drive(1.5,defaultDriveSpeed/2.5,1000);
            Strafe(46,defaultDriveSpeed,1000);
          
            /************
             * Path End *
             ***********/
        }

        /*********************************************************************
         *                              Next Path
         ********************************************************************/
        //Path Selection Error
        else {
            /************************************
             *          Path Selection Error
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

            /************
             * Path End *
             ***********/
        }

        // End of Paths
        sleep(30000); //Allow the timer to run to the end so that nothing else happens before the timer is up
    }


    /**********************
     * Path Methods
     **********************/
    public void park(boolean wall, boolean spin) throws InterruptedException{

        /*******
         *   Park after dropping at backdrop
         *   near the wall or away from it
         *   spin around and back in or not
         *******/

        //Strafe away from backdrop
        int parkDistance = -15;
        if (wall){
            parkDistance = 15;
        }
        Strafe(parkDistance*allianceReverser,defaultDriveSpeed,defaultPauseTime);

        //Spin around and pull in
        if(spin){
            Turn(180,defaultTurnSpeed,defaultPauseTime);
            Drive(-14, defaultDriveSpeed, defaultPauseTime);//Back to park area
        } else {
            Drive(14, defaultDriveSpeed, defaultPauseTime);//Forward to park area
        }

        wrist.setPosition(wristConversionToServo(55-armRotationDegreesAtHome));
        ReturnExtension();
        ReturnArm(); //Lower arm to floor

    }

    public void alignToAprilTag(int tagID, boolean leftOfCamera,boolean leftOfMountain, boolean leftPixelInScoop) throws InterruptedException {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        boolean stillAligning = true;
        double sweepCounter = 0;
        double maxSweep = 5;
        int sweepDirection = 1;
        double headingOffset = 0;
        DESIRED_TAG_ID = tagID;
        targetFound = false;
        desiredTag  = null;
        if (leftOfCamera){
            headingOffset = cameraBearingOffsetLeftTagLeftPixelLeftSide;
            if(!leftOfMountain) {
                headingOffset = headingOffset + distanceBetweenValleys;
            }

            if(!leftPixelInScoop){
                headingOffset = headingOffset - distanceBetweenScoopPositions;
                }
        } else {
            DESIRED_TAG_ID = tagID+1;
            headingOffset = cameraBearingOffsetRightTagRightPixelRightSide;
            if(leftOfMountain) {
                headingOffset = headingOffset - distanceBetweenValleys;
            }

            if(leftPixelInScoop){
                headingOffset = headingOffset + distanceBetweenScoopPositions;
            }
        }

        while (opModeIsActive())   // Loop to find the tag and drive to it
        {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Sweeping\n");
            }

            // If we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - mtzConstants_ItD.backdropAprilTagDESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing + mtzConstants_ItD.cameraBearingOffsetLeftTagLeftPixelLeftSide;
                double  yawError        = desiredTag.ftcPose.yaw;

                if (rangeError < mtzConstants_ItD.alignConfidence && headingError< mtzConstants_ItD.alignConfidence && yawError < mtzConstants_ItD.alignConfidence){

                    drive  = 0;
                    turn   = 0;
                    strafe = 0;
                    moveRobot(drive, strafe, turn);
                    return;
                }
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // turn to find tag
                if (Math.abs(sweepCounter)<maxSweep) {
                    turn = -.1 * sweepDirection;
                    sweepCounter = sweepCounter + sweepDirection;
                }
                else {
                    sweepDirection = -1 * sweepDirection;
                    turn = -.1 * sweepDirection;
                    sweepCounter = sweepCounter + sweepDirection;
                }
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
        telemetry.addLine("Something went wrong with the alignment");
        sleep(30000);
        return;
    }

    /**********************
     * Sampling Methods
     **********************/
/******


    public int determinePixelLocation () throws InterruptedException {
        int pixelLocation = 0;
        int pixelTimer = 10000;
        if (opModeIsActive()) {
            while (opModeIsActive() && pixelTimer>0) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                telemetry.addData("# Objects Detected", currentRecognitions.size());

                // Step through the list of recognitions and display info for each one.
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    if (x<100) {
                        pixelLocation = 1;
                    }
                    else if (x>=100 && x<200){
                        pixelLocation = 2;
                    }
                    else if (x>200){
                        pixelLocation = 3;
                    }
                }   // end for() loop

                // Share the CPU.
                sleep(20);
                pixelTimer=pixelTimer-20;
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        return pixelLocation;
    }
*******/

    public void colorSensePixelLocation (int alliance) throws InterruptedException {
        double colorThreshold = 300;
        //Drive to left spike
        String targetColor = "Blue";
        if (alliance<0){
            targetColor = "Red";
        }
        Drive(26,defaultDriveSpeed/2,defaultPauseTime);
        //Strafe(-7.5,defaultDriveSpeed/2,defaultPauseTime);
        Turn(-90*allianceReverser,defaultTurnSpeed,defaultPauseTime);
        Drive(8,defaultDriveSpeed/4,defaultPauseTime);
        //Sample Left Spike
        if (alliance<0){ //Alliance is Red
            if (rightColorSensor.red()>colorThreshold) {
                randomizerPosition = 1;
            }
        } else { //Alliance is Blue
            if (rightColorSensor.blue()>(colorThreshold-100)) {
                randomizerPosition = 1;
            }
            //Drive(-8,defaultDriveSpeed,defaultPauseTime);

        }
        if(randomizerPosition == 2){
            //backup, strafe right, forward
            //Drive(-4,defaultDriveSpeed,defaultPauseTime);
            //Strafe(4,defaultDriveSpeed/2,defaultPauseTime);
            Drive(-4,defaultDriveSpeed/2,defaultPauseTime);
            //Strafe(9,defaultDriveSpeed/2,defaultPauseTime);
            Turn(-180*allianceReverser,defaultTurnSpeed,defaultPauseTime);
            Drive(10,defaultDriveSpeed/4,defaultPauseTime);

            //Sample RightLef Spike
            if (alliance<0){ //Alliance is Red
                if (leftColorSensor.red()>colorThreshold) {
                    randomizerPosition = 3;
                }
            } else { //Alliance is Blue
                if (leftColorSensor.blue()>(colorThreshold-100)) {
                    randomizerPosition = 3;
                }
            }
            //return to center of sampling
            //Strafe(-4,defaultDriveSpeed/2,defaultPauseTime);
            Drive(-10,defaultDriveSpeed/4,defaultPauseTime);
        } else {
            //return to center of sampling
            //Strafe(4,defaultDriveSpeed/2,defaultPauseTime);
            //Turn(-90,defaultTurnSpeed,defaultPauseTime);
            //Drive(4,defaultDriveSpeed/4,defaultPauseTime);
        }
        if(randomizerPosition == 2){
            //backup, strafe right, forward
            //Drive(-4,defaultDriveSpeed,defaultPauseTime);
            //Strafe(4,defaultDriveSpeed/2,defaultPauseTime);
            Drive(-4,defaultDriveSpeed/2,defaultPauseTime);
            //Strafe(9,defaultDriveSpeed/2,defaultPauseTime);
            Turn(-90*allianceReverser,defaultTurnSpeed,defaultPauseTime);
            Drive(-6,defaultDriveSpeed/4,defaultPauseTime);
            Drive(4,defaultDriveSpeed/4,defaultPauseTime);

            //return to center of sampling
            //Strafe(-4,defaultDriveSpeed/2,defaultPauseTime);
            //Drive(-10,defaultDriveSpeed/4,defaultPauseTime);
        }

    }

    /**********************
     * Motion Methods
     **********************/

    public void Drive(double distance, double motorPower, int pause) throws InterruptedException {
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
    public void Strafe(double rightDistance, double power, int pause) throws InterruptedException {
        //Right is positive
        if (opModeIsActive()) {
            StopAndResetDriveEncoders();
            StrafeByInches(rightDistance);
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
            RaiseByDegrees(degrees);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmPower(defaultArmPower);
        }
        while (arm.isBusy()) {
            DisplayArmTelemetry();
        }
        ArmPower(0);
    }

    public void RaiseArm(int distance, int pause) throws InterruptedException {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set the target rotations for the motor
        RaiseByInches(distance);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmPower(defaultArmPower);
        while (arm.isBusy()) {
            DisplayArmTelemetry();
        }
    }
    public void ReturnArm() throws InterruptedException {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set the target rotations for the motor
        arm.setTargetPosition(-armOdometer);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmPower(defaultArmPower/2);
        while (arm.isBusy()) {
            DisplayArmTelemetry();
        }
        armOdometer=0;
    }
    public void ExtendArm(double additionalExtension, double power,int pause) throws InterruptedException {
        if (opModeIsActive()) {
            armExtension.setTargetPosition((int) (armExtension.getCurrentPosition() + (additionalExtension * ticksPerInchExtension)));
            armExtension.setPower(power);
            while (arm.isBusy() || armExtension.isBusy()) {
                DisplayArmTelemetry();
            }
            armExtension.setPower(0);
        }
        Thread.sleep(pause);
    }
    public void ReturnExtension() throws InterruptedException {
        if (opModeIsActive()) {
            armExtension.setTargetPosition(0);
            armExtension.setPower(defaultArmExtensionPower/2);
            while (arm.isBusy() || armExtension.isBusy()) {
                DisplayArmTelemetry();
            }
            armExtension.setPower(0);
        }
        extendOdometer=0;
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

    /**********************
     * Encoder Methods
     **********************/

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
    public void RunDriveWithEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void RunDriveWithOutEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void RunArmToPosition() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*********************************
     * Distance Calculation Methods
     ********************************/

    public void DriveByInches(double distance) {
        frontLeft.setTargetPosition((int)(distance * ticksPerInchWheelDrive));
        frontRight.setTargetPosition((int)(distance * ticksPerInchWheelDrive));
        backLeft.setTargetPosition((int)(-1 * distance * ticksPerInchWheelDrive));
        backRight.setTargetPosition((int)(-1 * distance * ticksPerInchWheelDrive));
    }
    public void StrafeByInches(double distance) {
        frontLeft.setTargetPosition((int)(distance * ticksPerInchWheelStrafe));
        frontRight.setTargetPosition((int)(-distance * ticksPerInchWheelStrafe));
        backLeft.setTargetPosition((int)(distance * ticksPerInchWheelStrafe));
        backRight.setTargetPosition((int)(-distance * ticksPerInchWheelStrafe));
    }
    public void TurnByAngle(double degrees) {
        frontLeft.setTargetPosition((int)(degrees * ticksPerDegreeTurnChassis));
        frontRight.setTargetPosition((int)(-degrees * ticksPerDegreeTurnChassis));
        backLeft.setTargetPosition((int)(-degrees * ticksPerDegreeTurnChassis));
        backRight.setTargetPosition((int)(degrees * ticksPerDegreeTurnChassis));
    }
    public void RaiseByInches(double distance) {
        int correctedDistance = (int) (distance * (armDistanceAdjustment));
        arm.setTargetPosition(correctedDistance);
        armOdometer=armOdometer+correctedDistance;
    }
    public void RaiseByDegrees(double degrees) {
        int correctedDistance = (int)(degrees * ticksPerDegreeArm);
        arm.setTargetPosition(correctedDistance);
        armOdometer=armOdometer+correctedDistance;
    }



    /**********************
     * Power Methods
     **********************/

    public void DrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void ArmPower(double power) {
        arm.setPower(power);
    }

    /******
     * TensorFlow Methods
     **********/

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    /********

    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()


     ****/

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

    /************


    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


    //End TensorFlow Methods
*********************************/


    /**********************
     * Telemetry Methods
     **********************/

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
        telemetry.addLine()
                .addData("Randomizer Position: ", randomizerPosition);
        angles=imuForDisplay.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        telemetry.addData("Heading: ",angles.firstAngle);
        telemetry.addData("Roll: ",angles.secondAngle);
        telemetry.addData("Pitch: ",angles.thirdAngle);
        telemetry.update();
    }
    public void DisplayArmTelemetry() {
        double armDegrees = arm.getCurrentPosition() / ticksPerDegreeArm;
        double extensionInches = armExtension.getCurrentPosition() / ticksPerInchExtension;
        double frontRightInches = frontRight.getCurrentPosition() / conversionTicksToInches;
        double backLeftInches = backLeft.getCurrentPosition() / conversionTicksToInches;
        double backRightInches = backRight.getCurrentPosition() / conversionTicksToInches;
        telemetry.clear();
        telemetry.addLine()
                .addData("Arm Degrees ", (int) armDegrees + "   Power: " + "%.1f", arm.getPower());
        telemetry.addLine()
                .addData("Arm Extension ", (int) extensionInches + "   Power: " + "%.1f", armExtension.getPower());
        telemetry.update();
    }

    /************************
     * Align w/AprilTag Methods
     */


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x +y -yaw;
        double rightFrontPower   =  x -y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    //End of Class
}
