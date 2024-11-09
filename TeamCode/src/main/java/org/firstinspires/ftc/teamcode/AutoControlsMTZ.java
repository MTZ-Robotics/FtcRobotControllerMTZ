package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.armRotationDegreesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstants_ItD.defaultArmExtensionPower;
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

@Autonomous(name ="Auto Controls", group = "Bottom")
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

public class AutoControlsMTZ extends LinearOpMode {


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


        /*****************************************************************************
         * Coding Instructions
         *
         * Write paths for Red alliance and apply reverser on turns and strafes
         *
         ****************************************************************************/


        if (pathToRun.contains("Calibrate")) {

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
            Strafe(allianceReverser*24,defaultDriveSpeed,1000);
            Drive(34,defaultDriveSpeed,1000);
            Turn(allianceReverser*89,defaultTurnSpeed,1000);
            Drive(6.5,defaultDriveSpeed,1000);
            Strafe(allianceReverser*46,defaultDriveSpeed,1000);
            Strafe(allianceReverser*-46,defaultDriveSpeed,1000);
            Drive(6.5,defaultDriveSpeed,1000);
            Strafe(allianceReverser*46,defaultDriveSpeed,1000);
            Strafe(allianceReverser*-48,defaultDriveSpeed,1000);
            Drive(7,defaultDriveSpeed,1000);
            Drive(1.5,defaultDriveSpeed/2.5,1000);
            Strafe(allianceReverser*48,defaultDriveSpeed,1000);

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

    /**********************
     * Sampling Methods
     **********************/



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

    /****
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
