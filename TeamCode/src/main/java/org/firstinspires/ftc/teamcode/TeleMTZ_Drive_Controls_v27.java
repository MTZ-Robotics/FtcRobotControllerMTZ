package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armExtensionCollapsedLength;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armExtensionInchesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armLengthDesired;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armPivotHeight;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.armRotationDegreesAtHome;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.blockThrowerDownPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.blockThrowerUpPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.clawClosedPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.clawOpenPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmLowerPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultPauseTime;
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
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftHookDownPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftHookInPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.leftHookUpPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.maxArmDegrees;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.maxArmExtensionInches;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.maxWristPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.minArmDegrees;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.minArmExtensionInches;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.minWristPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.redWarningTime;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightHookDownPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightHookInPosition;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.rightHookUpPosition;
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
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.wristBump;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.wristConversionToServo;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.yellowWarningTime;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleMTZ_Drive_Controls v27 Tarragon", group ="A_Top")

//Tarragon

//Adds run to position to controls map
//Adds control map error handling

//@Disabled

public class TeleMTZ_Drive_Controls_v27 extends LinearOpMode {

    /********************************
     * Robot Configuration Flags
     ********************************/
    boolean accountForArmDrift;
    boolean hasChassisMotors;
    boolean hasArmMotorsAndServos;
    boolean hasExpansionHubConnected;
    boolean hasLightsHub;
    boolean wantAutoChassisControls;

    /********************************
     * Timer Variables
     ********************************/
    private ElapsedTime endGameTimer;

    boolean greenTimerElapsed;
    boolean yellowTimerElapsed;
    boolean redTimerElapsed;
    boolean endGameStartElapsed;

    /***********
     * Lights Control Declarations
     ***********/

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    /*************************
     * Motor & Servo Variables
     *************************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor arm;
    private DcMotor armExtension;
    private Servo claw;
    private Servo leftHook;
    private Servo rightHook;
    private Servo blockThrower;
    private Servo wrist;

    double drivePower;

    boolean clawRemainClosed;
    boolean aboveLevel = false;
    boolean stackingDown;

    int stackLevel = stackLevelAtHome;
    int stackDistance = stackDistanceAtHome;
    double armRotationDegrees = armRotationDegreesAtHome;
    double armExtensionInches = armExtensionInchesAtHome;
    double verticalDesired;
    double horizontalDesired;
    double stackDegreesDesired;
    double wristPositionDesired = wristConversionToServo(90+armRotationDegreesAtHome);


    /*******
     * Add Button Variables & Objects
     ********/

    /*************           MTZ Controls Variables            **************/
// Assign Variables & Objects for Control Pads
    double chassisSpeedSlow;                             //Slow Speed
    mtzButtonBehavior chassisBumpLeftTurnStatus = new mtzButtonBehavior();         //Bump Left Turn




    mtzButtonBehavior chassisBumpForwardStatus = new mtzButtonBehavior();         //Bump Forward
    mtzButtonBehavior chassisBumpLeftStrafeStatus = new mtzButtonBehavior();         //Bump Left Strafe
    mtzButtonBehavior chassisBumpRightStrafeStatus = new mtzButtonBehavior();         //Bump Right Strafe
    mtzButtonBehavior chassisBumpBackStatus = new mtzButtonBehavior();         //Bump Backwards

    mtzButtonBehavior robotExtenderInButtonStatus = new mtzButtonBehavior();         //Park Extender In
    double driveStick1;                             //Drive 1
    double turnStick;                             //Turn
    double chassisSpeedFast;                             //High Speed
    mtzButtonBehavior chassisBumpRightTurnStatus = new mtzButtonBehavior();         //Bump Right Turn

    mtzButtonBehavior startButton1Status = new mtzButtonBehavior();         //Pad Select (A & B)


    mtzButtonBehavior hooksDownButtonStatus = new mtzButtonBehavior();         //Hooks Down
    mtzButtonBehavior hooksInButtonStatus = new mtzButtonBehavior();         //Hooks In

    mtzButtonBehavior blockThrowerButtonStatus = new mtzButtonBehavior();         //Block Thrower
    mtzButtonBehavior hooksUpButtonStatus = new mtzButtonBehavior();         //Hooks Up

    mtzButtonBehavior robotExtenderOutButtonStatus = new mtzButtonBehavior();         //Park Extender Out
    double driveStick2;                             //Drive 2
    double strafeStick;                             //Strafe
    double clawOpen;                             //Claw Open (Sticky)
    mtzButtonBehavior wristAdjustLessStatus = new mtzButtonBehavior();         //Wrist Adjust -

    mtzButtonBehavior resetAdjustmentsStatus = new mtzButtonBehavior();         //Reset Adjustments
    mtzButtonBehavior stoneReleaseButtonStatus = new mtzButtonBehavior();         //Release Stone

    mtzButtonBehavior stackLevelUpStatus = new mtzButtonBehavior();         //Stack Level Up
    mtzButtonBehavior stackDistanceLessStatus = new mtzButtonBehavior();         //Stack Distance -
    mtzButtonBehavior stackDistanceMoreStatus = new mtzButtonBehavior();         //Stack Distance +
    mtzButtonBehavior stackHalfLevelDownStatus = new mtzButtonBehavior();         //Stack Half Level Down


    double handVerticalStick;                             //Hand Vertical Move
    double handHorizontalStick;                             //Hand Horizontal Move
    double clawClose;                             //Claw Close (Sticky)
    mtzButtonBehavior wristAdjustMoreStatus = new mtzButtonBehavior();         //Wrist Adjust +

    mtzButtonBehavior startButton2Status = new mtzButtonBehavior();         //Pad Select (A & B)


    mtzButtonBehavior handAdjustHigherStatus = new mtzButtonBehavior();         //Hand Adjust Higher
    mtzButtonBehavior handAdjustInStatus = new mtzButtonBehavior();         //Hand Adjust In
    mtzButtonBehavior handAdjustOutStatus = new mtzButtonBehavior();         //Hand Adjust Out
    mtzButtonBehavior handAdjustLowerStatus = new mtzButtonBehavior();         //Hand Adjust Lower


    double handAssist;                             //Ride Height/Drop to 0
    double stoneReleaseStick;                             //Release Stone (RL Flick)
// End of Assignment Mapping
    /*************           End     MTZ Control Pad Variables            **************/


    @Override

    //This is the opMode call for generically running the opMode in this super class
    public void runOpMode() throws InterruptedException{


        String controlPadMap = "SkyStone Braves Left Strafe";
        boolean spurGearArm = false;
        double driveSpeed = 0.5;
        telemetry.log().add("Controls Map:"+controlPadMap+", Arm Support:"+spurGearArm+", Drive Power:"+driveSpeed);
        controlRobot(controlPadMap,spurGearArm,driveSpeed);
    }

    //This is the method that handles the controls
    public void controlRobot(String controlPadMap, Boolean supportArm, Double defaultDrivePower) throws InterruptedException {

        // Robot Configuration Flags
        accountForArmDrift = supportArm;
        hasChassisMotors = true;
        hasArmMotorsAndServos = true;
        hasExpansionHubConnected = true;
        hasLightsHub = true;
        wantAutoChassisControls = true;

        /***********************
         * Modifiable variables
         **********************/
        //mtzConstantsCS.endGameStart = 90;
        //endGameWarning = endGameStart + 15;
        //endGameOver = endGameStart + 30;
        //greenWarningTime = 60;
        //yellowWarningTime = 70;
        //redWarningTime = 80;
        //defaultArmPower = 0.75;

        /***************
         * Set Timer Variables
         ***************/
        greenTimerElapsed = false;
        yellowTimerElapsed = false;
        redTimerElapsed = false;
        endGameStartElapsed = false;

        /*************
         * Set Lights Variables
         *************/
       if(hasLightsHub) {
           blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

           pattern = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
           blinkinLedDriver.setPattern(pattern);
       }

        /*******************************
         * Set Motor & Servo Variables
         ******************************/
        if(hasChassisMotors){
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            backRight = hardwareMap.dcMotor.get("backRight");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);
        }
        if(hasArmMotorsAndServos){
            arm = hardwareMap.dcMotor.get("arm");
            armExtension = hardwareMap.dcMotor.get("armExtension");
            claw = hardwareMap.servo.get("leftClaw");
            wrist = hardwareMap.servo.get("rightClaw");
            //rightHook = hardwareMap.servo.get("rightHook");
            //leftHook = hardwareMap.servo.get("leftHook");
            //blockThrower = hardwareMap.servo.get("blockThrower");

            armExtension.setDirection(DcMotor.Direction.REVERSE);
            leftHook.setDirection(Servo.Direction.REVERSE);

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //arm.setDirection(DcMotor.Direction.REVERSE);
        }


        /**********************************
         * Do Not set positions on initialize
         **********************************/


        /***********************************************
         * Tell driver station that initialization complete
         **********************************************/
        telemetry.log().add("Initialized. Go MTZ! Be Sure to Home Arm. Timer Set for ");

        telemetry.log().add(greenWarningTime+" s, " +
                yellowWarningTime+" s, " +
                redWarningTime+" s, " +
                endGameStart+" s, "
        );

        /************* Press Play Button ***********************/

        waitForStart();
        if(hasLightsHub) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            blinkinLedDriver.setPattern(pattern);
        }
        //Start timer here since play was just pressed
        endGameTimer = new ElapsedTime();
        endGameTimer.reset();

        while (opModeIsActive()) {  //Loop often to see if controls are still the same



            /***********
             * Gather Button Input
             */

            if (controlPadMap=="SkyStone Braves Left Strafe") {

/*************           SkyStone Braves Left Strafe     Controls Update Status           **************/
                chassisSpeedSlow = gamepad1.left_trigger;             //Slow Speed
                chassisBumpLeftTurnStatus.update(gamepad1.left_bumper);             //Bump Left Turn




                chassisBumpForwardStatus.update(gamepad1.dpad_up);             //Bump Forward
                chassisBumpLeftStrafeStatus.update(gamepad1.dpad_left);             //Bump Left Strafe
                chassisBumpRightStrafeStatus.update(gamepad1.dpad_right);             //Bump Right Strafe
                chassisBumpBackStatus.update(gamepad1.dpad_down);             //Bump Backwards

                robotExtenderInButtonStatus.update(gamepad1.left_stick_button);             //Park Extender In
                driveStick2 = gamepad1.left_stick_y;             //Drive 2
                strafeStick = gamepad1.left_stick_x;             //Turn
                chassisSpeedFast = gamepad1.right_trigger;             //High Speed
                chassisBumpRightTurnStatus.update(gamepad1.right_bumper);             //Bump Right Turn

                startButton1Status.update(gamepad1.start);             //Pad Select (A & B)


                hooksUpButtonStatus.update(gamepad1.y);             //Hooks Up
                hooksInButtonStatus.update(gamepad1.x);             //Hooks In
                blockThrowerButtonStatus.update(gamepad1.b);             //Block Thrower
                hooksDownButtonStatus.update(gamepad1.a);             //Hooks Down

                robotExtenderOutButtonStatus.update(gamepad1.left_stick_button);             //Park Extender Out
                driveStick1 = gamepad1.right_stick_y;             //Drive 1
                strafeStick = gamepad1.right_stick_x;             //Strafe
                clawOpen = gamepad2.left_trigger;             //Claw Open (Sticky)
                wristAdjustLessStatus.update(gamepad2.left_bumper);             //Wrist Adjust -

                resetAdjustmentsStatus.update(gamepad2.guide);             //Reset Adjustments
                stoneReleaseButtonStatus.update(gamepad2.back);             //Release Stone

                stackLevelUpStatus.update(gamepad2.dpad_up);             //Stack Level Up
                stackDistanceLessStatus.update(gamepad2.dpad_left);             //Stack Distance -
                stackDistanceMoreStatus.update(gamepad2.dpad_right);             //Stack Distance +
                stackHalfLevelDownStatus.update(gamepad2.dpad_down);             //Stack Half Level Down


                handVerticalStick = gamepad2.left_stick_y;             //Hand Vertical Move
                handHorizontalStick = gamepad2.left_stick_x;             //Hand Horizontal Move
                clawClose = gamepad2.right_trigger;             //Claw Close (Sticky)
                wristAdjustMoreStatus.update(gamepad2.right_bumper);             //Wrist Adjust +

                startButton2Status.update(gamepad2.start);             //Pad Select (A & B)


                handAdjustHigherStatus.update(gamepad2.y);             //Hand Adjust Higher
                handAdjustInStatus.update(gamepad2.x);             //Hand Adjust In
                handAdjustOutStatus.update(gamepad2.b);             //Hand Adjust Out
                handAdjustLowerStatus.update(gamepad2.a);             //Hand Adjust Lower


                handAssist = gamepad2.right_stick_y;             //Ride Height/Drop to 0
                stoneReleaseStick = gamepad2.right_stick_x;             //Release Stone (RL Flick)
/*************           End     SkyStone Braves Left Strafe     Updates            **************/
            } else if (controlPadMap=="SkyStone Braves Right Strafe") {

/*************           SkyStone Braves Right Strafe     Controls Update Status           **************/
                chassisSpeedSlow = gamepad1.left_trigger;             //Slow Speed
                chassisBumpLeftTurnStatus.update(gamepad1.left_bumper);             //Bump Left Turn




                chassisBumpForwardStatus.update(gamepad1.dpad_up);             //Bump Forward
                chassisBumpLeftStrafeStatus.update(gamepad1.dpad_left);             //Bump Left Strafe
                chassisBumpRightStrafeStatus.update(gamepad1.dpad_right);             //Bump Right Strafe
                chassisBumpBackStatus.update(gamepad1.dpad_down);             //Bump Backwards

                robotExtenderInButtonStatus.update(gamepad1.left_stick_button);             //Park Extender In
                driveStick1 = gamepad1.left_stick_y;             //Drive 1
                turnStick = gamepad1.left_stick_x;             //Turn
                chassisSpeedFast = gamepad1.right_trigger;             //High Speed
                chassisBumpRightTurnStatus.update(gamepad1.right_bumper);             //Bump Right Turn

                startButton1Status.update(gamepad1.start);             //Pad Select (A & B)


                hooksUpButtonStatus.update(gamepad1.y);             //Hooks Up
                hooksInButtonStatus.update(gamepad1.x);             //Hooks In
                blockThrowerButtonStatus.update(gamepad1.b);             //Block Thrower
                hooksDownButtonStatus.update(gamepad1.a);             //Hooks Down

                robotExtenderOutButtonStatus.update(gamepad1.left_stick_button);             //Park Extender Out
                driveStick2 = gamepad1.right_stick_y;             //Drive 2
                strafeStick = gamepad1.right_stick_x;             //Strafe
                clawOpen = gamepad2.left_trigger;             //Claw Open (Sticky)
                wristAdjustLessStatus.update(gamepad2.left_bumper);             //Wrist Adjust -

                resetAdjustmentsStatus.update(gamepad2.guide);             //Reset Adjustments
                stoneReleaseButtonStatus.update(gamepad2.back);             //Release Stone

                stackLevelUpStatus.update(gamepad2.dpad_up);             //Stack Level Up
                stackDistanceLessStatus.update(gamepad2.dpad_left);             //Stack Distance -
                stackDistanceMoreStatus.update(gamepad2.dpad_right);             //Stack Distance +
                stackHalfLevelDownStatus.update(gamepad2.dpad_down);             //Stack Half Level Down


                handVerticalStick = gamepad2.left_stick_y;             //Hand Vertical Move
                handHorizontalStick = gamepad2.left_stick_x;             //Hand Horizontal Move
                clawClose = gamepad2.right_trigger;             //Claw Close (Sticky)
                wristAdjustMoreStatus.update(gamepad2.right_bumper);             //Wrist Adjust +

                startButton2Status.update(gamepad2.start);             //Pad Select (A & B)


                handAdjustHigherStatus.update(gamepad2.y);             //Hand Adjust Higher
                handAdjustInStatus.update(gamepad2.x);             //Hand Adjust In
                handAdjustOutStatus.update(gamepad2.b);             //Hand Adjust Out
                handAdjustLowerStatus.update(gamepad2.a);             //Hand Adjust Lower


                handAssist = gamepad2.right_stick_y;             //Ride Height/Drop to 0
                stoneReleaseStick = gamepad2.right_stick_x;             //Release Stone (RL Flick)
/*************           End     SkyStone Braves Right Strafe     Updates            **************/
            } else if (controlPadMap=="SkyStone Right Strafe") {

                /*************           SkyStone Right Strafe     Controls Update Status           **************/
                chassisSpeedSlow = gamepad1.left_trigger;             //Slow Speed












                strafeStick = gamepad1.left_stick_x;             //Strafe
                chassisSpeedFast = gamepad1.right_trigger;             //High Speed











                driveStick1 = gamepad1.right_stick_y;             //Drive
                turnStick = gamepad1.right_stick_x;             //Turn

                wristAdjustLessStatus.update(gamepad2.left_bumper);             //Wrist Adjust +




                hooksDownButtonStatus.update(gamepad2.dpad_up);             //Hooks Up


                hooksUpButtonStatus.update(gamepad2.dpad_down);             //Hooks Down


                handVerticalStick = gamepad2.left_stick_y;             //Arm Angle
                handHorizontalStick = gamepad2.left_stick_x;             //Arm Extension

                mtzButtonBehavior wristAdjustMoreStatus = new mtzButtonBehavior();         //Wrist Adjust -







                blockThrowerButtonStatus.update(gamepad2.a);             //Block Thrower


                clawClose = gamepad2.right_stick_y;             //Claw

                /*************           End     SkyStone Right Strafe     Updates            **************/
            } else if (controlPadMap=="SkyStone Left Strafe") {

                /*************           SkyStone Left Strafe     Controls Update Status           **************/
                chassisSpeedSlow = gamepad1.left_trigger;             //Slow Speed











                driveStick1 = gamepad1.left_stick_y;             //Drive
                turnStick = gamepad1.left_stick_x;             //Turn
                chassisSpeedFast = gamepad1.right_trigger;             //High Speed












                strafeStick = gamepad1.right_stick_x;             //Strafe

                wristAdjustLessStatus.update(gamepad2.left_bumper);             //Wrist Adjust +




                hooksDownButtonStatus.update(gamepad2.dpad_up);             //Hooks Up


                hooksUpButtonStatus.update(gamepad2.dpad_down);             //Hooks Down


                handVerticalStick = gamepad2.left_stick_y;             //Arm Angle
                handHorizontalStick = gamepad2.left_stick_x;             //Arm Extension

                wristAdjustMoreStatus.update(gamepad2.right_bumper);             //Wrist Adjust -







                blockThrowerButtonStatus.update(gamepad2.a);             //Block Thrower


                clawClose = gamepad2.right_stick_y;             //Claw

                /*************           End     SkyStone Left Strafe     Updates            **************/
            } else {
                /************************************
                 * Control Pad Map Selection Error
                 ***********************************/
                telemetry.log().add("Error in Control Map Selection"); telemetry.update();
                if(hasLightsHub){pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED; blinkinLedDriver.setPattern(pattern);}
                waitForStart(); sleep(30000);
            }

            displayTelemetry();

            /*********************
             * Speed adjust with triggers
             ********************/
            if (chassisSpeedFast > 0) {
                drivePower = defaultDrivePower * driveFastRatio;
            } else if (chassisSpeedSlow > 0) {
                drivePower = defaultDrivePower * driveSlowRatio;
                } else {
                    drivePower = defaultDrivePower;
            }

            /*****************
             * Hook Controls
             ****************/

            if (hooksDownButtonStatus.clickedDown) { HooksDown(); }
            if (hooksUpButtonStatus.clickedDown) { HooksUp(); }

            /*************************
             * Chassis drive controls
             *************************/
            if(hasChassisMotors) {
                //Set motors to run manually
                backLeft.setPower(drivePower * ((driveStick1 + strafeStick) - turnStick));
                backRight.setPower(drivePower * ((driveStick1 - strafeStick) + turnStick));
                frontLeft.setPower(drivePower * ((-driveStick1 + strafeStick) + turnStick));
                frontRight.setPower(drivePower * ((-driveStick1 - strafeStick) - turnStick));
            }
            /*************************
             * Chassis bump controls
             *************************/
            if(hasChassisMotors && wantAutoChassisControls) {
                if (chassisBumpForwardStatus.clickedDown) {
                    Drive(driveBump, .5, 0);
                }
                if (chassisBumpBackStatus.clickedDown) {
                    Drive(driveBump, .5, 0);
                }
                if (chassisBumpLeftStrafeStatus.clickedDown) {
                    Strafe(strafeBump, .5, 0);
                }
                if (chassisBumpRightStrafeStatus.clickedDown) {
                    Strafe(-strafeBump, .5, 0);
                }
                if (chassisBumpLeftTurnStatus.clickedDown) {
                    Turn(-turnBump, .5, 0);
                }
                if (chassisBumpRightTurnStatus.clickedDown) {
                    Turn(turnBump, .5, 0);
                }
            }

            /*************
             * Arm Controls
             *************/
            if(hasArmMotorsAndServos) {
                if (accountForArmDrift) {
                    arm.setPower(-1 * (defaultArmPower * (handVerticalStick) - 0.2));
                } else {

                    if (handVerticalStick < 0) {
                        arm.setPower(defaultArmPower * (-handVerticalStick));
                    } else {
                        arm.setPower(defaultArmLowerPower * (-handVerticalStick));
                    }
                }

                armExtension.setPower((handHorizontalStick));
            }
            if (handVerticalStick!=0){
                stackLevel = -1;
            }
            if (handHorizontalStick!=0){
                stackDistance = -1;
            }

            //handAssist
            if(handAssist<=-0.9){
                //Ride Height Desired
                stackLevel = handAssistRideHeightLevel;
                stackDistance = handAssistRideHeightDistance;
                aboveLevel = handAssistRideHeightAboveLevel;
                goToStackPosition(false,stackLevel,stackDistance,aboveLevel);
            }
            if(handAssist>=0.9){
                // Zero Height Desired
                stackLevel = 0;
                stackDistance = 0;
                aboveLevel = false;
                goToStackPosition(false,stackLevel,stackDistance,aboveLevel);
            }

            /************************
             * Stacker Controls
             ***********************/
            if(hasArmMotorsAndServos) {
                armRotationDegrees = (arm.getCurrentPosition() / ticksPerDegreeArm) + armRotationDegreesAtHome;
                armExtensionInches = armExtension.getCurrentPosition() / ticksPerInchExtension - armExtensionInchesAtHome;
            }

            if(stackLevelUpStatus.clickedDown){
                stackingDown=false;
                if(stackLevel!=-1 && stackLevel < stackHeightOnLevelArray.length-1){
                    stackLevel++;
                }
                goToStackPosition(stackingDown,stackLevel,stackDistance,aboveLevel);
            }
            if(stackHalfLevelDownStatus.clickedDown){
                stackingDown=true;
                if(stackLevel!=-1){
                    if(aboveLevel){
                        aboveLevel = false;
                    } else if(stackLevel!=0){
                        stackLevel--;
                        aboveLevel = true;
                    }
                }
                goToStackPosition(stackingDown,stackLevel,stackDistance,aboveLevel);
            }
            if(stackDistanceMoreStatus.clickedDown){
                stackingDown = false;
                if(stackDistance!=-1 && stackDistance < stackHeightOnLevelArray.length-1){
                    stackDistance++;
                }
                goToStackPosition(stackingDown,stackLevel,stackDistance,aboveLevel);
            }
            if(stackDistanceLessStatus.clickedDown){
                stackingDown = false;
                if(stackDistance>0 && stackDistance < stackHeightOnLevelArray.length-1){
                    stackDistance--;
                }
                goToStackPosition(stackingDown,stackLevel,stackDistance,aboveLevel);
            }


            /*************
             * Claw Controls
             *************/

            if(clawClose>0.95){clawRemainClosed = true; }
            if(clawOpen>0.95){clawRemainClosed = false;}
            if(hasArmMotorsAndServos) {
                if (clawRemainClosed) {
                    claw.setPosition(clawClosedPosition);
                } else {
                    claw.setPosition(clawOpenPosition - clawClose - clawOpen);
                }
            }

            /************************
             * Cap Stone thrower controls
             ***********************/
            if(hasArmMotorsAndServos) {
                if (blockThrowerButtonStatus.isDown) {
                    blockThrower.setPosition(blockThrowerDownPosition);
                } else {
                    blockThrower.setPosition(blockThrowerUpPosition);
                }
            }

            /*************
             * Wrist Controls
             *************/
            if (wristAdjustLessStatus.clickedDown) {
                wristPositionDesired = wristPositionDesired - wristBump;
            } else if (wristAdjustMoreStatus.clickedDown) {
                wristPositionDesired = wristPositionDesired + wristBump;
            }

            if(wristPositionDesired < minWristPosition){
                wristPositionDesired = minWristPosition;
            }

            if(wristPositionDesired > maxWristPosition){
                wristPositionDesired = maxWristPosition;
            }
            //Set wrist position
            if(hasArmMotorsAndServos) {
                wrist.setPosition(wristPositionDesired);
            }

            /*********************************
             * Check if timer has elapsed
             *********************************/
            if(hasLightsHub) {
                //Check for End Timer First
                if (endGameTimer.seconds() > endGameOver) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > endGameWarning2) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > endGameWarning) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > endGameStart) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > redWarningTime) { //Then check for red
                    redTimerElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > yellowWarningTime) { //Then check for yellow
                    yellowTimerElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > greenWarningTime) { //Then check for green
                    greenTimerElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    blinkinLedDriver.setPattern(pattern);
                }
            }
        }
    }

    /*******************************
     * End of Control Robot Method
     ******************************/

    //Motion Methods

    public void Drive(double distance, double motorPower, int pause) throws InterruptedException {
        if(hasChassisMotors) {
            if (opModeIsActive()) {
                StopAndResetDriveEncoders();
                DriveByInches(distance);
                RunDriveToPosition();
                DrivePower(motorPower);
                while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    DisplayDriveTelemetry();
                }
                Thread.sleep(pause);
            }
        }
    }
    public void Strafe(double leftDistance, double power, int pause) throws InterruptedException {
        if(hasChassisMotors) {
            //Left is positive
            if (opModeIsActive()) {
                StopAndResetDriveEncoders();
                StrafeByInches(leftDistance);
                RunDriveToPosition();
                DrivePower(power);
                while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    DisplayDriveTelemetry();
                }
                Thread.sleep(pause);
            }
        }
    }
    public void Turn(double rightDegrees, double power, int pause) throws InterruptedException {
        if(hasChassisMotors) {
            //Left is negative
            if (opModeIsActive()) {
                StopAndResetDriveEncoders();
                TurnByAngle(rightDegrees);
                RunDriveToPosition();
                DrivePower(power);
                while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    DisplayDriveTelemetry();
                }
                Thread.sleep(pause);
            }
        }
    }
    public void goToStackPosition(boolean stackingDown,int stackLevel,int stackDistance,boolean aboveLevel) throws InterruptedException {
        double vertDesired, horDesired, degreesDesired, vertRequired,armSpeed;
        if (stackLevel<0){
            stackLevel = findStackLevel();
            if(stackingDown){
                aboveLevel = true;
            }
        }
        if (stackDistance<0){
            stackDistance = findStackDistance();
        }
        if(stackLevel>stackHeightOnLevelArray.length-1){
            stackLevel = stackHeightOnLevelArray.length-1;
        }
        if(stackDistance>stackDistanceArray.length-1){
            stackDistance = stackDistanceArray.length-1;
        }
        if (stackLevel >= 0 && stackDistance >= 0) {
            // Check if the stone is getting set down on a level and go slow if so
            if (aboveLevel) {
                vertDesired = stackHeightAboveLevelArray[stackLevel];
                armSpeed = defaultArmLowerPower;
            } else {
                vertDesired = stackHeightOnLevelArray[stackLevel];
                armSpeed = defaultArmLowerPower / 3;
            }
            if (!stackingDown) {
                armSpeed = defaultArmPower;
            }

            vertRequired = vertDesired - armPivotHeight;
            horDesired = stackDistanceArray[stackDistance];

            degreesDesired = Math.toDegrees(Math.asin((vertRequired) / (armLengthDesired(horDesired, vertDesired))));

            //Stay in the max & min
            armExtensionInches = armLengthDesired(horDesired, vertDesired) - armExtensionCollapsedLength;
            if(armExtensionInches < minArmExtensionInches){
                armExtensionInches = minArmExtensionInches;
            } else if(armExtensionInches > maxArmExtensionInches){
                armExtensionInches = maxArmExtensionInches;
            }
            if(degreesDesired < minArmDegrees){
                degreesDesired = minArmDegrees;
            } else if(degreesDesired > maxArmDegrees){
                degreesDesired = maxArmDegrees;
            }
            stackDegreesDesired = degreesDesired;
            // Set the target positions to run to
            raiseByDegrees(degreesDesired);
            wristPositionDesired = wristAutoLevel(degreesDesired);
            horizontalDesired = horDesired;
            verticalDesired = vertDesired;
            if(hasArmMotorsAndServos) {
                armExtension.setTargetPosition((int) ((armExtensionInches - armExtensionInchesAtHome) * ticksPerInchExtension));

                if (opModeIsActive()) {
                    // Turn motors on to let them reach the target if the stop button hasn't been pressed
                    arm.setPower(armSpeed);
                    armExtension.setPower(defaultArmExtensionPower);
                    //Wrist Position is set once so it doesn't try to go to 2 different positions each loop iteration
                    // wrist.setPosition(wristPositionDesired);
                    while (arm.isBusy() || armExtension.isBusy()) {
                        DisplayArmTelemetry();
                    }
                }

            }
            Thread.sleep(defaultPauseTime);
        }
    }

    public void RaiseArm(double degrees, double power,int pause) throws InterruptedException {
        if(hasArmMotorsAndServos) {
            if (opModeIsActive()) {
                raiseByDegrees(degrees);
                ArmPower(power);
                while (arm.isBusy() || armExtension.isBusy()) {
                    DisplayArmTelemetry();
                }
            }
            Thread.sleep(pause);
        }
    }
    public void LowerArm(double degrees, double power, int pause) throws InterruptedException {
        if(hasArmMotorsAndServos) {
            if (opModeIsActive()) {
                raiseByDegrees(-degrees);
                ArmPower(power);
                while (arm.isBusy() || armExtension.isBusy()) {
                    DisplayArmTelemetry();
                }
            }
            Thread.sleep(pause);
        }
    }
    public double wristAutoLevel(double armAngle){
                return wristConversionToServo(armAngle + 90);
    }

    public void ExtendArm(double desiredArmLength, double power,int pause) throws InterruptedException {
        if (opModeIsActive()) {
            armExtensionInches = desiredArmLength - armExtensionCollapsedLength;
            if(armExtensionInches<minArmExtensionInches){
                armExtensionInches=minArmExtensionInches;
            } else if(armExtensionInches>maxArmExtensionInches){
                armExtensionInches=maxArmExtensionInches;
            }
            if(hasArmMotorsAndServos) {
                armExtension.setTargetPosition((int) (armExtensionInches * ticksPerInchExtension));
                armExtension.setPower(power);

                while (arm.isBusy() || armExtension.isBusy()) {
                    DisplayArmTelemetry();
                }
            }
        }
        Thread.sleep(pause);
    }
    public void HooksDown()throws InterruptedException {
        if(hasArmMotorsAndServos) {
            //Light Reverse Power On
            leftHook.setPosition(leftHookDownPosition);
            rightHook.setPosition(rightHookDownPosition);
            //lightReverse();
            //sleep(1500);

            //Reverse Power Off
            //StopAndResetDriveEncoders();
        }
    }
    public void HooksUp() {
        if(hasArmMotorsAndServos) {
            leftHook.setPosition(leftHookUpPosition);
            rightHook.setPosition(rightHookUpPosition);
            //sleep(1500);
        }
    }
    public void HooksIn() {
        if(hasArmMotorsAndServos) {
            leftHook.setPosition(leftHookInPosition);
            rightHook.setPosition(rightHookInPosition);
            //sleep(500);
        }
    }
    public void lightForward() throws InterruptedException{
        if (hasChassisMotors) {
            Drive(1,0.1,50);
        }
    }


//Encoder Methods

    public void StopAndResetAllEncoders() {
        if(hasChassisMotors) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(hasArmMotorsAndServos){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void StopAndResetDriveEncoders() {
        if(hasChassisMotors) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void StopAndResetArmEncoder() {
        if(hasArmMotorsAndServos){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void RunDriveToPosition() {
        if(hasChassisMotors) {
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void RunArmToPosition() {
        if(hasArmMotorsAndServos) {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void RunArm() {
        if(hasArmMotorsAndServos) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //End of Encoder Methods

//Distance Calculation Methods

    public void DriveByInches(double distance) {
        if(hasChassisMotors) {
            frontLeft.setTargetPosition((int) (distance * ticksPerInchWheelDrive));
            frontRight.setTargetPosition((int) (distance * ticksPerInchWheelDrive));
            backLeft.setTargetPosition((int) (-1 * distance * ticksPerInchWheelDrive));
            backRight.setTargetPosition((int) (-1 * distance * ticksPerInchWheelDrive));
        }
    }

    public void StrafeByInches(double distance) {
        if(hasChassisMotors) {
            frontLeft.setTargetPosition((int) (distance * ticksPerInchWheelStrafe));
            frontRight.setTargetPosition((int) (-distance * ticksPerInchWheelStrafe));
            backLeft.setTargetPosition((int) (distance * ticksPerInchWheelStrafe));
            backRight.setTargetPosition((int) (-distance * ticksPerInchWheelStrafe));
        }
    }

    public void TurnByAngle(double degrees) {
        if(hasChassisMotors) {
            frontLeft.setTargetPosition((int) (degrees * ticksPerDegreeTurnChassis));
            frontRight.setTargetPosition((int) (-degrees * ticksPerDegreeTurnChassis));
            backLeft.setTargetPosition((int) (-degrees * ticksPerDegreeTurnChassis));
            backRight.setTargetPosition((int) (degrees * ticksPerDegreeTurnChassis));
        }
    }

    public void raiseByDegrees(double degrees) {
        if(hasArmMotorsAndServos){
            arm.setTargetPosition((int)((degrees - armRotationDegreesAtHome) * ticksPerDegreeArm));
        }
    }

    //End of distance calculation methods

//Power Methods

    public void DrivePower(double power) {
        if(hasChassisMotors) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
    }
    public void ArmPower(double power) {
        if(hasArmMotorsAndServos) {
            arm.setPower(power);
        }
    }
//End Power Methods


//Telemetry Methods
    public void displayTelemetry() {
        telemetry.clearAll();
        telemetry.addLine()
                .addData("Timer: ", endGameTimer.toString());
        telemetry.addLine()
                .addData("Stack Level: ", stackLevel);
        telemetry.addLine()
                .addData("Stack Distance: ", stackDistance);
        telemetry.addLine()
                .addData("Vert Desired: ", verticalDesired);
        telemetry.addLine()
                .addData("Hor Desired: ", horizontalDesired);
        telemetry.addLine()
                .addData("Arm Theoretical Degrees: ", armRotationDegrees);
        telemetry.addLine()
                .addData("Stack Degrees Desired: ", stackDegreesDesired);
        telemetry.addLine()
                .addData("Arm Length: ", armExtensionInches + armExtensionCollapsedLength);
        telemetry.addLine()
                .addData("ticksPerDegreesArm: ", ticksPerDegreeArm);
        telemetry.addLine()
                .addData("ticksPerInchExtension: ", ticksPerInchExtension);
        if(hasChassisMotors && hasArmMotorsAndServos) {
            telemetry.addLine()
                    .addData("Front Left Power: ", frontLeft.getPower());
            telemetry.addLine()
                    .addData("Front Right Power: ", frontRight.getPower());
            telemetry.addLine()
                    .addData("Back Left Power: ", backLeft.getPower());
            telemetry.addLine()
                    .addData("Back Right Power: ", backRight.getPower());
            telemetry.addLine()
                    .addData("Claw Position: ", claw.getPosition());
            telemetry.addLine()
                    .addData("Wrist Position: ", wrist.getPosition());
            telemetry.addLine()
                    .addData("Left Hook Position: ", leftHook.getPosition());
            telemetry.addLine()
                    .addData("Right Hook Position: ", rightHook.getPosition());
            telemetry.addLine()
                    .addData("Block Thrower Position: ", blockThrower.getPosition());
            telemetry.addLine()
                    .addData("Arm Extension: ", armExtension.getCurrentPosition());
            telemetry.addLine()
                    .addData("Arm Ticks: ", arm.getCurrentPosition());
            telemetry.addLine()
                    .addData("Arm Current Degrees: ", arm.getCurrentPosition() / ticksPerDegreeArm);
        }
        telemetry.update();
    }
    public void DisplayDriveTelemetry() {
        if(hasChassisMotors) {
            double frontLeftInches = frontLeft.getCurrentPosition() / ticksPerInchWheelDrive;
            double frontRightInches = frontRight.getCurrentPosition() / ticksPerInchWheelDrive;
            double backLeftInches = backLeft.getCurrentPosition() / ticksPerInchWheelDrive;
            double backRightInches = backRight.getCurrentPosition() / ticksPerInchWheelDrive;
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
    }
    public void DisplayArmTelemetry() {
        if(hasArmMotorsAndServos) {
            double armDegrees = arm.getCurrentPosition() / ticksPerDegreeArm;
            telemetry.clear();
            telemetry.addLine()
                    .addData("Arm Degrees ", (int) armDegrees + "  Power: " + "%.1f", arm.getPower());
            telemetry.addLine()
                    .addData("Arm Ext Inches ", (int) armExtensionInches + "  Power: " + "%.1f", armExtension.getPower());
            telemetry.update();
        }
    }
    //End of Telemetry Methods
    //End of Class
}
