package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Arm Raise Simple", group = "z_test")

//@Disabled              // Comment out to allow this OpMode to display in phone

public class z_test_arm_raise_simple extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    //private Servo leftHook;
    //private Servo rightHook;
    //private Servo blockThrower;

    double drivePower;

    /************
     *
     * Arm EWC Declarations
     *
     */

    private DcMotor arm;
    private DcMotor armExtension;
    private Servo claw;
    private Servo wrist;
    double armPower;
    double armExtensionPower;
    double calculatedWristPosition;
    double wristIncrement;


    @Override

    public void runOpMode() {

        /*************
         * Test arm using rear wheels only
         *
         *
         frontLeft = hardwareMap.dcMotor.get("frontLeft");
         frontRight = hardwareMap.dcMotor.get("frontRight");

        rightHook = hardwareMap.servo.get("rightHook");
        leftHook = hardwareMap.servo.get("leftHook");
        blockThrower = hardwareMap.servo.get("blockThrower");

        */

        /***********No chassis Motors, yet
         *
         *
         *
         *
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
         *
         */

        /************
         *
         * Arm EWC Objects
         *
         */

        armPower = 0.5;
        armExtensionPower = 1;
        arm = hardwareMap.dcMotor.get("arm");
        armExtension = hardwareMap.dcMotor.get("armExtension");
        //claw = hardwareMap.servo.get("claw");
        //wrist = hardwareMap.servo.get("wrist");

        arm.setDirection(DcMotor.Direction.REVERSE);
        //leftHook.setDirection(Servo.Direction.REVERSE);

        //claw.setPosition(0);


        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //backRight.setDirection(DcMotor.Direction.REVERSE);
        //leftHook.setPosition(0.5);
        //rightHook.setPosition(0.5);
        //blockThrower.setPosition(1);

        telemetry.log().add("Initialized. Test ARM!!!");

        waitForStart();

        while (opModeIsActive()) {

            displayTelemetry();


            if (gamepad1.right_trigger > 0) {
                drivePower = 1;
            } else if (gamepad1.left_trigger > 0) {
                drivePower = 0.25;
                } else {
                    drivePower = 0.5;
            }

            //backLeft.setPower(drivePower * ((gamepad1.right_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x));
            //backRight.setPower(drivePower * ((gamepad1.right_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x));
            //frontLeft.setPower(drivePower * ((-gamepad1.right_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x));
            //frontRight.setPower(drivePower * ((-gamepad1.right_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x));



            /**********
             *
             * Arm EWC Controls
             */


            /************
             *
             * Don't know speed needs, yet
             if (gamepad2.right_trigger > 0) {
             armPower = 0.5;
             } else if (gamepad2.left_trigger > 0) {
             armPower = 0.2;
             } else {
             armPower = 0.35;
             }

             *
             */

            //old arm power  arm.setPower(armPower * (gamepad2.left_stick_y) - 0.2);
            arm.setPower(armPower * gamepad2.left_stick_y);
            armExtension.setPower(armExtensionPower * gamepad2.left_stick_x);

            //raise arm slowly
            //arm.setPower(-1);
            //lower arm slowly
            //arm.setPower(0.7);




            /***********
             *
             * No Servos, Yet
             *
             *
            claw.setPosition(gamepad2.right_stick_y);

            calculatedWristPosition = wrist.getPosition();
            if (gamepad2.dpad_down) {
                calculatedWristPosition = calculatedWristPosition - wristIncrement;
            }
            if (gamepad2.dpad_up) {
                calculatedWristPosition = calculatedWristPosition + wristIncrement;
            }
             *
             *
             *
             */

            /****
            if (gamepad2.dpad_up) {
                leftHook.setPosition(0.5);
                rightHook.setPosition(0.5);
            }

            if (gamepad2.dpad_down) {
                leftHook.setPosition(0);
                rightHook.setPosition(0);
            }
            if (gamepad2.dpad_up) {
                leftHook.setPosition(0.5);
                rightHook.setPosition(0.5);
            }
            if(gamepad2.a){
                blockThrower.setPosition(0.45);
            } else {
                blockThrower.setPosition(1);
            }
            */
        }
    }

    public void displayTelemetry() {
        telemetry.clear();
        //telemetry.addLine()
        //        .addData("Back Left Power: ", backLeft.getPower());
        //telemetry.addLine()
        //        .addData("Back Right Power: ", backRight.getPower());
        telemetry.addLine()
                .addData("Arm Power: ", arm.getPower());
        telemetry.addLine()
                .addData("Arm Ext Power: ", armExtension.getPower());
        //telemetry.addLine()
        //        .addData("Claw Position: ", claw.getPosition());
        telemetry.update();
    }

}
