package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftClawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightClawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeArm;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeTurnChassis;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerRevolution1150;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name ="Auto Controls v114", group = "Bottom")
//@Disabled

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
 *
 *******************/

public class AutoControlsMTZv114 extends LinearOpMode {


    /**************
     *
     * Modify these speeds to help with diagnosing drive errors
     *
     **************/
    private static final double defaultDriveSpeed = 0.1;
    private static final double defaultTurnSpeed = 0.1;
    private static int defaultPauseTime = 300;

    /**********************
     * These variables are the constants in path commands
     **********************/
    private static final double ticksPerRevolution = ticksPerRevolution1150;
    private static final double gearReduction = 1.0;
    private static final double wheelDiameterInches = 4.0;
    private static final double pi = 3.1415;
    private static final double conversionTicksToInches = (ticksPerRevolution * gearReduction) / (pi * wheelDiameterInches);
    private static final double armDistanceAdjustment = 39.4;
    private static final double strafeDistanceAdjustment = 1.15;

    private static final double driveDistanceAdjustment = .85;
    private int allianceReverser = 1;


    public int armOdometer=0;
    public int extendOdometer=0;


    /******************
     * April Tag Alignment Declarations
     */

    private static final int DESIRED_TAG_ID = 3;     // Choose the tag you want to approach or set to -1 for ANY tag.
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
    private DcMotor flywheel;
    //private Servo claw;
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
    private TfodProcessor tfod;


    //End TensorFlow Set-up



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
            pathToRun="Backdrop Sample";
        }

        /*****************************************************************************
         * Coding Instructions
         *
         * Write paths for Blue alliance and apply reverser on turns and strafes
         *
         ****************************************************************************/


        if (pathToRun == "Backdrop" || pathToRun == "Audience" || pathToRun == "Backdrop Sample" || pathToRun == "Audience Sample" || pathToRun == "Audience Wall" || pathToRun == "Backdrop Wall"){
            /******************************************************************
             *                           Path Branch 4
             *****************************************************************/

            Logging.log("Running Path Branch 4");

            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Line up notes should be entered in. ");
            waitForStart();

            //sleep(1000);

            double ExtraDistance = 0;
            if (pathToRun == "Audience" || pathToRun == "Audience Wall"){
                ExtraDistance = 54;
            }


            RaiseArm(3,defaultPauseTime);



            Drive(24, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area
            Drive(-24, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area

            Turn(90*allianceReverser,defaultTurnSpeed, defaultPauseTime); //Turn towards the backdrop
            Strafe(15*allianceReverser, defaultDriveSpeed, defaultPauseTime); //head towards wall
            Strafe(3*allianceReverser, defaultDriveSpeed/2, defaultPauseTime); //straighten up on the wall



            Strafe(-4*allianceReverser, defaultDriveSpeed, defaultPauseTime); //move away from the wall
            Drive(24 + ExtraDistance, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area
            Strafe(-22*allianceReverser, defaultDriveSpeed, defaultPauseTime); //slide in front of backdrop. Strafe isn't working so we just took this one out.
            Turn(5*allianceReverser,defaultDriveSpeed,defaultPauseTime); //To look towards rest of field
            //Drive(26, defaultDriveSpeed, defaultPauseTime); //To get in front of backdrop
            //Turn(90,defaultDriveSpeed,defaultPauseTime); //To look towards backdrop


            //slide over in front of the april tag for the drop

            //defaultPauseTime=defaultPauseTime+1000;


            RaiseArm(14,defaultPauseTime); //Raise arm
            ExtendArm(-24, defaultArmExtensionPower, defaultPauseTime);
            Drive(20, defaultDriveSpeed, defaultPauseTime);
            Drive(8, defaultDriveSpeed/4, defaultPauseTime);
            //Deliver Pixel


            //ExtendArm(5,defaultArmExtensionPower,defaultPauseTime); //extend arm
            leftClaw.setPosition(leftClawOpenPosition); //release left pixel
            rightClaw.setPosition(rightClawOpenPosition); //release left pixel
            RaiseArm(1,defaultPauseTime); //raise arm a little more
            ReturnExtension(); //retract arm

            //Park

            Drive(-4, defaultDriveSpeed, defaultPauseTime); //Back up a little

            int parkDistance = -25;
            if (pathToRun=="Audience Wall" || pathToRun == "Backdrop Wall"){
                parkDistance = 26;
            }

            Strafe(parkDistance*allianceReverser,defaultDriveSpeed,defaultPauseTime);
            ReturnArm(); //Lower arm to floor
            Drive(14, defaultDriveSpeed, defaultPauseTime);//Forward to park area



        }

        else if (pathToRun == "Backdrop Align") {

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


            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            if (USE_WEBCAM)
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();


            //End Copy


            waitForStart();



            RaiseArm(3,defaultPauseTime);



            Drive(24, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area
            Drive(-24, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area

            Turn(90*allianceReverser,defaultTurnSpeed, defaultPauseTime); //Turn towards the backdrop
            Strafe(15*allianceReverser, defaultDriveSpeed, defaultPauseTime); //head towards wall
            Strafe(3*allianceReverser, defaultDriveSpeed/2, defaultPauseTime); //straighten up on the wall



            Strafe(-4*allianceReverser, defaultDriveSpeed, defaultPauseTime); //move away from the wall
            Drive(24, defaultDriveSpeed, defaultPauseTime); //Travel back to backdrop area
            Strafe(-22*allianceReverser, defaultDriveSpeed, defaultPauseTime); //slide in front of backdrop. Strafe isn't working so we just took this one out.
            Turn(5*allianceReverser,defaultDriveSpeed,defaultPauseTime); //To look towards rest of field
            //Drive(26, defaultDriveSpeed, defaultPauseTime); //To get in front of backdrop
            //Turn(90,defaultDriveSpeed,defaultPauseTime); //To look towards backdrop


            //slide over in front of the april tag for the drop


            //Copied from AutoAlign

            boolean stillAligning = true;
            double sweepCounter = 0;
            double maxSweep = 5;
            int sweepDirection = 1;
            while (opModeIsActive())   // Loop to find the tag and drive to it
            {
                targetFound = false;
                desiredTag  = null;

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
                    double  rangeError      = (desiredTag.ftcPose.range - mtzConstantsCS.backdropAprilTagDESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing + mtzConstantsCS.cameraBearingOffsetLeftTagLeftPixelLeftSide;
                    double  yawError        = desiredTag.ftcPose.yaw;

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



            //End Copy

        }
        else if (pathToRun == "ArmTest") {

                /******************************************************************
                 *                           Path Branch Arm Test
                 *****************************************************************/

                Logging.log("Running Path Branch Arm Test");
                /************************************
                 * Path set up -- Add to each path
                 ***********************************/
                //Robot Setup Notes
                telemetry.log().add("Line up notes should be entered in. ");
                waitForStart();

            //arm.setPower();

            RaiseArm(1, defaultPauseTime);
            sleep(20000);


        }

        else if (pathToRun=="Calibrate") {

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
            RaiseArm(10,2000);
            Drive(24,defaultDriveSpeed,5000);
            Strafe(-24,defaultDriveSpeed,10000);
            Turn(-180,defaultTurnSpeed,0);
            ExtendArm(10, defaultArmExtensionPower,2000);
            sleep(20000);
            ExtendArm(-10, defaultArmExtensionPower,2000);
            RaiseArm(-10,2000);
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
    public void goToFoundationfromWall(int allianceReverser) throws InterruptedException{

        //Align Hooks With Foundation
        Drive(24, defaultDriveSpeed, defaultPauseTime);
        Strafe(allianceReverser * -12, defaultDriveSpeed, defaultPauseTime);
        Drive(5, defaultDriveSpeed, defaultPauseTime);
    }
    public void moveFoundation (int allianceReverser) throws InterruptedException{

        //Hook Foundation
        //HooksDown();

        //Move Foundation to Build Zone
        Drive(-20, 0.2, defaultPauseTime);
        Turn(allianceReverser * 40, 0.2, defaultPauseTime);
        Drive(5, -0.2, defaultPauseTime);
        Turn(allianceReverser * 80, 0.2, defaultPauseTime);
        Strafe(allianceReverser * 5, 0.2, defaultPauseTime);
        Drive(12, 0.1, defaultPauseTime);

        //Unhook Foundation
        //HooksUp();
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
    public void grabSkyStone(int allianceReverser) throws InterruptedException {
        //Angle towards skystone
        //CloseClaw();
        //claw.setPosition(mtzConstants.clawClosedPosition);
        //Wait for it to close
        sleep(1000);
        //Raise Arm
        RaiseArm(4,defaultPauseTime);
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
        //Left is positive
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
            RaiseByInches (degrees);
            ArmPower(defaultArmPower);
        }
        ArmPower(0);
        Thread.sleep(pause);

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

    public void DriveGyro(double inches, double motorPower, int pause) throws InterruptedException {
        /****
         * inches: how far the robot is supposed to travel
         * power: how strong the motors should be targeting
         *
         */

        double targetDistance = inches * conversionTicksToInches*driveDistanceAdjustment;
        int i = 0;
        double splitDistance;
        angles = imuForDisplay.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double originalHeading = angles.firstAngle;

        //double orientationGain = 1*(10^-15);
        double orientationGain = 0.00000001;
        int stepJump = 100;
        while(opModeIsActive()) {
            while (targetDistance > 0) {
                if (targetDistance > stepJump) {
                    //split the target distance up so error can be adjusted
                    if (i < 3) {
                        splitDistance = stepJump - 20;
                    } else {
                        splitDistance = stepJump;
                    }
                } else {
                    //use up all of the target distance and go slow
                    splitDistance = (targetDistance);
                    // Reduce the motor speeds on each motor
                    motorPower = 0.2 * motorPower;
                }


                //Tell the motors the intended distance
                frontLeft.setTargetPosition((int) (splitDistance));
                frontRight.setTargetPosition((int) (splitDistance));
                backLeft.setTargetPosition((int) (splitDistance));
                backRight.setTargetPosition((int) (splitDistance));

                // Only drive to the distance for one motor,
                // otherwise the other motors will still want to finish their count
                // and throw the orientation wacky at the end

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                /*
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                 */

                //Get the current orientation
                angles = imuForDisplay.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                //CCW is Positive for the orientation
                double orientationError = originalHeading - angles.firstAngle;

                //Set the motor power based on the error noted on the imu with gain

                //Subtract Error on Left
                frontLeft.setPower(motorPower - (orientationError * orientationGain));
                backLeft.setPower(motorPower - (orientationError * orientationGain));

                //Add error on right
                frontRight.setPower(motorPower + (orientationError * orientationGain));
                backRight.setPower(motorPower + (orientationError * orientationGain));


                //increase the counters for the next iteration of the loop
                targetDistance = targetDistance - splitDistance;
                double frontLeftInches = frontLeft.getCurrentPosition();
                double frontRightInches = frontRight.getCurrentPosition();
                double backLeftInches = backLeft.getCurrentPosition();
                double backRightInches = backRight.getCurrentPosition();

                while (frontLeft.isBusy()) {
                    /*
                    telemetry.clear();

                    telemetry.addLine()
                            .addData("Front Left Ticks ", (int) frontLeftInches + "   Power: " + "%.1f", frontLeft.getPower());
                    telemetry.addLine()
                            .addData("Front Right Ticks: ", (int) frontRightInches + "   Power: " + "%.1f", frontRight.getPower());
                    telemetry.addLine()
                            .addData("Back Left Ticks: ", (int) backLeftInches + "   Power: " + "%.1f", backLeft.getPower());
                    telemetry.addLine()
                            .addData("Back Right Ticks: ", (int) backRightInches + "   Power: " + "%.1f", backRight.getPower());


                     */
                    telemetry.addData("Target Distance: ", targetDistance);
                    telemetry.addData("Original Heading: ", originalHeading);
                    telemetry.addData("Orientation Error: ", orientationError);
                    telemetry.addData("Loop Iteration: ", i);
                    telemetry.update();
                }
                //DisplayDriveTelemetry();
                //StopAndResetDriveEncoders();
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                //wait for humans to read
                sleep(1000);

                i = i + 1;
            }
            Thread.sleep(pause);
        }
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
    public void RunArmToPosition() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*********************************
     * Distance Calculation Methods
     ********************************/

    public void DriveByInches(double distance) {
        double correctedDistance = (distance*(driveDistanceAdjustment));
        frontLeft.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        frontRight.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        backLeft.setTargetPosition((int)(-1 * correctedDistance * conversionTicksToInches));
        backRight.setTargetPosition((int)(-1 * correctedDistance * conversionTicksToInches));
    }
    public void StrafeByInches(double distance) {
        double correctedDistance = distance*(strafeDistanceAdjustment);
        frontLeft.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        frontRight.setTargetPosition((int)(-correctedDistance * conversionTicksToInches));
        backLeft.setTargetPosition((int)(correctedDistance * conversionTicksToInches));
        backRight.setTargetPosition((int)(-correctedDistance * conversionTicksToInches));
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
    public void raiseByDegrees(double degrees) {
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
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
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
