package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armRotationDegreesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.cameraBearingOffsetLeftTagLeftPixelLeftSide;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.cameraBearingOffsetRightTagRightPixelRightSide;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.distanceBetweenScoopPositions;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.distanceBetweenValleys;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.randomizerPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightClawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeArm;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeTurnChassis;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchWheelDrive;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchWheelStrafe;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerRevolution1150;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.wristConversionToServo;

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

import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name ="Auto Controls Push Bot", group = "Bottom")
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

public class AutoControlsMTZ_PushBot extends LinearOpMode {


    /**************
     *
     * Modify these speeds to help with diagnosing drive errors
     *
     **************/
    private static final double defaultDriveSpeed = 0.2;
    private static final double defaultTurnSpeed = 0.2;
    private static int defaultPauseTime = 100;

    /**********************
     * These variables are the constants in path commands
     **********************/
    private static final double ticksPerRevolution = ticksPerRevolution1150;
    private static final double gearReduction = 1.0;
    private static final double wheelDiameterInches = 4.0;

    private static final double pi = 3.1415;
    private static final double conversionTicksToInches = (ticksPerRevolution * gearReduction) / (pi * wheelDiameterInches);
    private static final double armDistanceAdjustment = 39.4;





    /*****************
     * Declare motor & servo objects
     ****************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;


    /**************
     * Sampling variables
     */

    int randomNumberResult = 2;


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




        /********
         * Movement starts here on initialize
         */


        //This was commented out in the code that was running in meet 1
        //Leaving it in to see if the arm behaves better after being reset
        StopAndResetAllEncoders();

        //}
        telemetry.log().clear();
        telemetry.update();
        telemetry.log().add(pathToRun+" Initialized. Go "+alliance+" alliance");



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
            pathToRun="auto2024";
        }

        double distanceBetweenStartingPositions = 0;

        //if (pathToRun == "Audience" || pathToRun == "Audience Wall" || pathToRun == "Audience Sample" || pathToRun == "Audience Align"){distanceBetweenStartingPositions = 54;}
        //Replaced the code above to try to simplify code
        if(pathToRun.contains("Audience")){
            distanceBetweenStartingPositions = 54;
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
            //RaiseArmByDegrees(10,2000);
            Drive(24,defaultDriveSpeed/3,5000);
            Strafe(-24,defaultDriveSpeed/2,5000);
            Turn(-180,defaultTurnSpeed,0);
            //ExtendArm(10, defaultArmExtensionPower,2000);
            sleep(2000);
            //RaiseArmByDegrees(90,2000);
            sleep(2000);
            //ExtendArm(-10, defaultArmExtensionPower,2000);
            //RaiseArmByDegrees(armRotationDegreesAtHome,2000);
            /************
             * Path End *
             ***********/
        }

        else if (pathToRun.contains("auto2024")) {

            /******************************************************************
             *                           Path Branch Calibrate
             *****************************************************************/

            Logging.log("Running Path Auto 2024");

            /************************************
             * Path set up -- Add to each path
             ***********************************/
            //Robot Setup Notes
            telemetry.log().add("Robot Starts on human player side of center");
            waitForStart();
            /************
             * Path Start
             ************/

            Drive(24,defaultDriveSpeed/3,5000);
            Strafe(-24,defaultDriveSpeed/2,5000);
            Turn(-180,defaultTurnSpeed,0);
            ////ExtendArm(10, defaultArmExtensionPower,2000);
            sleep(2000);
            //RaiseArmByDegrees(90,2000);
            sleep(2000);
            //ExtendArm(-10, defaultArmExtensionPower,2000);
            //RaiseArmByDegrees(armRotationDegreesAtHome,2000);
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


    /**********************
     * Encoder Methods
     **********************/

    public void StopAndResetAllEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void StopAndResetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    /**********************
     * Power Methods
     **********************/

    public void DrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
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
        telemetry.update();
    }

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

    //End of Class
}
