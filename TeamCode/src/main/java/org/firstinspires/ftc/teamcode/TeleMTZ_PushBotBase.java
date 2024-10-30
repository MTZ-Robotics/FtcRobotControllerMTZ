package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armExtensionCollapsedLength;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armExtensionInchesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armLengthDesired;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armPivotHeight;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armRotationDegreesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.cameraBearingOffsetLeftTagLeftPixelLeftSide;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.cameraBearingOffsetRightTagRightPixelRightSide;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmAssistLevel;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmLowerPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultDriveSpeed;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultPauseTime;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.distanceBetweenScoopPositions;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.distanceBetweenValleys;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.driveBump;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.driveFastRatio;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.driveSlowRatio;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.endGameOver;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.endGameStart;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.endGameWarning;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.endGameWarning2;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.findStackDistance;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.findStackLevel;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.greenWarningTime;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.handAssistRideHeightAboveLevel;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.handAssistRideHeightDistance;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.handAssistRideHeightLevel;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.launcherReleasePosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.launcherSetPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftClawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.maxArmDegrees;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.maxArmExtensionInches;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.maxWristPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.minArmDegrees;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.minArmExtensionInches;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.minWristPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.prorate;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.redWarningTime;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightClawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightClawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.scoopStage;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.stackDistanceArray;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.stackDistanceAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.stackHeightAboveLevelArray;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.stackHeightOnLevelArray;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.stackLevelAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.strafeBump;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeArm;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerDegreeTurnChassis;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchWheelDrive;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchWheelStrafe;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.turnBump;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.wristAdjustment;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.wristBump;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.wristConversionToServo;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.yellowWarningTime;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="TeleMTZ_PushBotBase", group ="Test")

//@Disabled

/****
 * This class is intended to be a sub class to run the robot with the controllers.
 * The super classes may call out different controller maps to use
 *
 *
 * v100 Copied from Last Year
 * v101 Updates during meet 1
 * v102
 * v103
 * v104 Added Power Ratio to slow arms with wheels
 * v105
 * v106 removed sticky from drive speed adjust with triggers
 * v107 Added Auto Raise to Hang
 * v108 Added AprilTag Alignment back in
 * v109 Moved Launcher to Y
 *
 *
 */

public class TeleMTZ_PushBotBase extends LinearOpMode {

    /********************************
     * Robot Configuration Flags
     ********************************/
    boolean accountForArmDrift;
    boolean hasChassisMotors;
    boolean hasAuxMotorsAndServos;
    boolean hasLightsHub;
    boolean wantAutoChassisControls;
    boolean leftClawRemainClosed = true;
    boolean rightClawRemainClosed = true;
    private int allianceReverser = 1;
    double armAssistLevel = defaultArmAssistLevel;
    double launcherPosition = launcherSetPosition;

    /********************************
     * Timer Variables
     ********************************/
    private ElapsedTime endGameTimer;

    boolean greenTimerElapsed;
    boolean yellowTimerElapsed;
    boolean redTimerElapsed;
    boolean endGameStartElapsed;
    double tempLightsTimer;


    /*************************
     * Motor & Servo Variables
     *************************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;

    double drivePower;
    double blPower;
    double brPower;
    double flPower;
    double frPower;
    double powerRatio=1;

    boolean aboveLevel = false;
    boolean stackingDown;

    int tagAdditional = 0;

    int stackLevel = stackLevelAtHome;
    int stackDistance = stackDistanceAtHome;
    double armRotationDegrees = armRotationDegreesAtHome;
    double armExtensionInches = armExtensionInchesAtHome;
    double verticalDesired;
    double horizontalDesired;
    double stackDegreesDesired;
    String debugString ="none";


    double wristPositionDesired = wristConversionToServo(armRotationDegreesAtHome) + wristAdjustment;


    /*******
     * Add Controller Variables & Objects
     ********/


    /****Blue*********           Center Stage R1     Control Pad Map            *******Blue*******/
// Assign Variables & Objects for Control Pads
    double chassisSpeedSlow;                             //Slow Speed
    mtzButtonBehavior chassisBumpLeftTurnStatus = new mtzButtonBehavior();         //Bump Left Turn




    mtzButtonBehavior chassisBumpForwardStatus = new mtzButtonBehavior();         //Bump Forward
    mtzButtonBehavior chassisBumpLeftStrafeStatus = new mtzButtonBehavior();         //Bump Left Strafe
    mtzButtonBehavior chassisBumpRightStrafeStatus = new mtzButtonBehavior();         //Bump Right Strafe
    mtzButtonBehavior chassisBumpBackStatus = new mtzButtonBehavior();         //Bump Backwards


    double driveStick2;                             //Drive 2
    double turnStick;                             //Turn
    double chassisSpeedFast;                             //High Speed
    mtzButtonBehavior chassisBumpRightTurnStatus = new mtzButtonBehavior();         //Bump Right Turn

    mtzButtonBehavior startButton1Status = new mtzButtonBehavior();         //Pad Select (A & B)


    mtzButtonBehavior aprilTagCenterStatus = new mtzButtonBehavior();         //Aim to Center AprilTag
    mtzButtonBehavior aprilTagLeftStatus = new mtzButtonBehavior();         //Aim to Left AprilTag
    mtzButtonBehavior aprilTagRightStatus = new mtzButtonBehavior();         //Aim to Right AprilTag
    mtzButtonBehavior planeLaunchStatus = new mtzButtonBehavior();         //Launch Plane


    double driveStick1;                             //Drive 1
    double strafeStick;                             //Strafe
    double leftClawClose;                             //Left Claw Close (Sticky)
    mtzButtonBehavior leftClawOpenStatus = new mtzButtonBehavior();         //Left Claw Open (Sticky)




    mtzButtonBehavior levelUpStatus = new mtzButtonBehavior();         //Move Hand to Next Level Higher
    mtzButtonBehavior wristAdjustLessStatus = new mtzButtonBehavior();         //Slightly Decrease Wrist
    mtzButtonBehavior wristAdjustMoreStatus = new mtzButtonBehavior();         //Slightly Increase Wrist
    mtzButtonBehavior levelDownStatus = new mtzButtonBehavior();         //Move Hand to Next Level Lower


    double handVerticalStick;                             //Hand Vertical Move
    double handHorizontalStick;                             //Hand Horizontal Move
    double rightClawClose;                             //Right Claw Close (Sticky)
    mtzButtonBehavior rightClawOpenStatus = new mtzButtonBehavior();         //Right Claw Open (Sticky)

    mtzButtonBehavior startButton2Status = new mtzButtonBehavior();         //Pad Select (A & B)


    mtzButtonBehavior raiseToHangStatus = new mtzButtonBehavior();         //Raise to Hang
    mtzButtonBehavior resetHomeStatus = new mtzButtonBehavior();         //Reset Home Position (With Start)
    mtzButtonBehavior rideHeightStatus = new mtzButtonBehavior();         //Ride Height
    mtzButtonBehavior returnHomeStatus = new mtzButtonBehavior();         //Drop to Home


    double armExtensionStick;                             //Arm Extension Stick

// End of Assignment Mapping
    /****Blue*********           End     Center Stage R1     Control Pad Map            ******Blue********/



    @Override

    //This is the default opMode call for generically running the opMode in this class directly from the phone without calling it from a super class
    public void runOpMode() throws InterruptedException{
        /******************************************************
         * These default settings will be used if THIS opMode is selected from the driver station.
         * Typically this opMode is not called since this class is used by super classes to call the controlRobot method with specific variables.
         * It is helpful to use this opMode for testing the controlRobot method
         *****************************************************/

        controlRobot("Red", "Center Stage R1", defaultDriveSpeed, true, true, true, true);
    }

    //This is the method that handles the controls
    public void controlRobot(String alliance, String controlPadMap, Double defaultDrivePower, Boolean runChassis, Boolean runAux, Boolean runLights, Boolean runAutos) throws InterruptedException {

        // Robot Configuration Flags
        hasChassisMotors = runChassis;
        hasAuxMotorsAndServos = runAux;
        hasLightsHub = runLights;
        wantAutoChassisControls = runAutos;
        // non-counterbalanced Spur Gear arm needs to account for arm drift
        accountForArmDrift = false;

        //Code written for Blue alliance and reverse turns if on Red alliance
        allianceReverser=1;
        if (alliance=="Red") {
            allianceReverser=-1;
        }

        /***********************
         * Modifiable variables
         **********************/

        /***************
         * reset Timer Variables to false
         ***************/
        greenTimerElapsed = false;
        yellowTimerElapsed = false;
        redTimerElapsed = false;
        endGameStartElapsed = false;

        /*************
         * Assign Lights Variables
         *************/


        /*******************************
         * Assign Motor & Servo Variables
         ******************************/
        if(hasChassisMotors){
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            backRight = hardwareMap.dcMotor.get("backRight");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }



        /**********************************
         * Do Not set positions on initialize since that counts as controlling the robot
         * and initialize would not be able to happen until the timer starts for driver controlled period
         **********************************/


        /***********************************************
         * Tell driver station that initialization is complete
         **********************************************/
        telemetry.log().clear();
        telemetry.update();
        telemetry.log().add("Initialized. Control Pad Map = "+controlPadMap+". Go "+alliance+" alliance");

        telemetry.log().add(greenWarningTime+", " +
                yellowWarningTime+", " +
                redWarningTime+", " +
                endGameStart
        );

        wristAdjustment = 0.0;
        /************* Press Play Button ***********************/

        waitForStart();

        /************ START ***************/


        //Start timer here since play was just pressed
        endGameTimer = new ElapsedTime();
        endGameTimer.reset();

        while (opModeIsActive()) {
            /**************************************************************
             *
             * TeleOp Loops From Here to the End of controlRobot
             *
             * Loops often to see if controls are still the same
             *
             ****************************************************************/

            /***********************
             * Gather Button Input *
             **********************/


/****Green*********           Center Stage R1     Controls Update Status           *******Green*******/

                chassisSpeedSlow = gamepad1.left_trigger;             //Slow Speed
                chassisBumpLeftTurnStatus.update(gamepad1.left_bumper);             //Bump Left Turn




                chassisBumpForwardStatus.update(gamepad1.dpad_up);             //Bump Forward
                chassisBumpLeftStrafeStatus.update(gamepad1.dpad_left);             //Bump Left Strafe
                chassisBumpRightStrafeStatus.update(gamepad1.dpad_right);             //Bump Right Strafe
                chassisBumpBackStatus.update(gamepad1.dpad_down);             //Bump Backwards


                driveStick2 = gamepad1.left_stick_y;             //Drive 2
                turnStick = gamepad1.left_stick_x;             //Turn
                chassisSpeedFast = gamepad1.right_trigger;             //High Speed
                chassisBumpRightTurnStatus.update(gamepad1.right_bumper);             //Bump Right Turn

                startButton1Status.update(gamepad1.start);             //Pad Select (A & B)


                planeLaunchStatus.update(gamepad1.y);             //Aim to Center AprilTag
                aprilTagLeftStatus.update(gamepad1.x);             //Aim to Left AprilTag
                aprilTagRightStatus.update(gamepad1.b);             //Aim to Right AprilTag
                aprilTagCenterStatus.update(gamepad1.a);             //Launch Plane


                driveStick1 = gamepad1.right_stick_y;             //Drive 1
                strafeStick = gamepad1.right_stick_x;             //Strafe
                leftClawClose = gamepad2.left_trigger;             //Left Claw Close (Sticky)
                leftClawOpenStatus.update(gamepad2.left_bumper);             //Left Claw Open (Sticky)




                levelUpStatus.update(gamepad2.dpad_up);             //Move Hand to Next Level Higher
                wristAdjustLessStatus.update(gamepad2.dpad_left);             //Slightly Decrease Wrist
                wristAdjustMoreStatus.update(gamepad2.dpad_right);             //Slightly Increase Wrist
                levelDownStatus.update(gamepad2.dpad_down);             //Move Hand to Next Level Lower


                handVerticalStick = gamepad2.left_stick_y;             //Hand Vertical Move
                handHorizontalStick = gamepad2.left_stick_x;             //Hand Horizontal Move
                rightClawClose = gamepad2.right_trigger;             //Right Claw Close (Sticky)
                rightClawOpenStatus.update(gamepad2.right_bumper);             //Right Claw Open (Sticky)

                startButton2Status.update(gamepad2.start);             //Pad Select (A & B)


                raiseToHangStatus.update(gamepad2.y);             //Raise to Hang
                resetHomeStatus.update(gamepad2.x);             //Reset Home Position (With Start)
                rideHeightStatus.update(gamepad2.b);             //Ride Height
                returnHomeStatus.update(gamepad2.a);             //Drop to Home


                armExtensionStick = gamepad2.right_stick_y;             //Arm Extension Stick






            /**********************************************************************************************
             * Speed adjust with triggers                                                                 *
             * If the chassisSpeedFast trigger is pulled, the motor speed will increase a constant amount *
             * If the chassisSpeedSlow trigger is pulled, the motor speed will decrease a constant amount *
             * chassisSpeedFast overrides chassisSpeedSlow                                                *
             *********************************************************************************************/
            if (chassisSpeedFast > 0) {
                powerRatio = driveFastRatio;
            } else if (chassisSpeedSlow > 0) {
                powerRatio = driveSlowRatio;
            } else {

                powerRatio = 1.0;
            }
            drivePower = defaultDrivePower*powerRatio;

            /**************************
             * Chassis drive controls *
             *************************/

            turnStick = turnStick * .85; //Turns are too fast, was 1.0
            blPower = drivePower * ((-driveStick2 + -driveStick1 + strafeStick) - turnStick);
            brPower = drivePower * ((-driveStick2 + -driveStick1 - strafeStick) + turnStick);
            flPower = drivePower * ((-driveStick2 + -driveStick1 + strafeStick) + turnStick);
            frPower = drivePower * ((-driveStick2 + -driveStick1 - strafeStick) - turnStick);




                //Set motors to run manually from controller
                backLeft.setPower(blPower);
                backRight.setPower(brPower);
                frontLeft.setPower(flPower);
                frontRight.setPower(frPower);


        }
    }

    /*******************************
     * End of Control Robot Method
     ******************************/


//Power Methods

    public void DrivePower(double power) {
        if(hasChassisMotors) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
    }

//End Power Methods
    //End of Class
}
