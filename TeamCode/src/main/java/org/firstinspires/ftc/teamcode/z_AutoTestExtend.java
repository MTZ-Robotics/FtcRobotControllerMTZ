package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerRevolution1150;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.sql.SQLOutput;

@Autonomous(name ="Auto Test Extend", group = "Bottom")
//@Disabled

/*************************
 * This class should be used to test the armExtension
 *******************/

public class z_AutoTestExtend extends LinearOpMode {


    /**************
     *
     * Modify these speeds to help with diagnosing drive errors
     *
     **************/
    private static int defaultPauseTime = 300;
    private static double defaultArmExtensionPower = .5;

    /**********************
     * These variables are the constants in path commands
     **********************/

    private int allianceReverser = 1;


    public int armOdometer=0;
    public int extendOdometer=0;

    /*****************
     * Declare motor & servo objects
     ****************/
     private DcMotor armExtension;



    mtzButtonBehavior boxClawSpacingStatus = new mtzButtonBehavior();         //Box Claw Spacing
    mtzButtonBehavior ballClawSpacingStatus = new mtzButtonBehavior();         //Ball Claw Spacing
    mtzButtonBehavior duckClawSpacingStatus = new mtzButtonBehavior();         //Duck Claw Spacing
    mtzButtonBehavior openClawSpacingStatus = new mtzButtonBehavior();         //Open Claw


    @Override

    public void runOpMode() throws InterruptedException {
        armExtension = hardwareMap.dcMotor.get("armExtension");
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setDirection(DcMotor.Direction.REVERSE);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.log().add("Should Extend 10 Inches, wait 1 second, then retract back at half speed. A Forward, B Reverse");
        boxClawSpacingStatus.update(gamepad2.y);             //Box Claw Spacing
        ballClawSpacingStatus.update(gamepad2.x);             //Ball Claw Spacing
        duckClawSpacingStatus.update(gamepad2.b);             //Duck Claw Spacing
        openClawSpacingStatus.update(gamepad2.a);             //Open Claw

        if (duckClawSpacingStatus.clickedDown){
            armExtension.setDirection((DcMotorSimple.Direction.REVERSE));
            telemetry.log().add("B");
        }
        if (openClawSpacingStatus.clickedDown){
            armExtension.setDirection((DcMotorSimple.Direction.FORWARD));
            telemetry.log().add("A");
        }

        telemetry.addData("Motor Direction",armExtension.getDirection());


        waitForStart();

        StopAndResetAllEncoders();

        armOdometer = 0;
        extendOdometer = 0;

        StopAndResetAllEncoders();

        //ExtendArm(10, defaultArmExtensionPower, defaultPauseTime);
        telemetry.addData("Motor Direction",armExtension.getDirection());
        telemetry.update();

        sleep(1000);
        telemetry.addLine("Extend Arm ");
        telemetry.update();
        sleep(1000);
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksTarget = armExtension.getCurrentPosition() + (.03 * ticksPerInchExtension);
        telemetry.addData("TicksTarget",ticksTarget);
        telemetry.update();
        sleep(1000);
        armExtension.setTargetPosition((int) (ticksTarget));
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtension.setPower(defaultArmExtensionPower);
        while (armExtension.isBusy()) {
            DisplayArmTelemetry();
        }
        sleep(1000);
        telemetry.addLine("Stop Extend Arm ");
        telemetry.update();
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setPower(0);
        sleep(1000);
        telemetry.addLine("Arm Extended");
        telemetry.update();
        sleep(1000);
        //ReturnExtension(); //retract arm
        telemetry.addLine("Retract Arm ");
        telemetry.update();
        sleep(1000);
        armExtension.setTargetPosition(0);
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtension.setPower(defaultArmExtensionPower/2);
        while (armExtension.isBusy()) {
            DisplayArmTelemetry();
        }
        sleep(1000);

        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Arm Retracted");
        telemetry.update();
        //armExtension.setPower(0);
        sleep(20000);
    }

    /**********************
     * Motion Methods
     **********************/


    public void ExtendArm(double additionalExtension, double power,int pause) throws InterruptedException {
        if (opModeIsActive()) {
            telemetry.addLine("opp mode active");
            telemetry.update();
            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            StopAndResetAllEncoders();
            armExtension.setTargetPosition((int) (armExtension.getCurrentPosition() + (additionalExtension * ticksPerInchExtension)));

            armExtension.setPower(power);
            while (armExtension.isBusy()) {
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
            while (armExtension.isBusy()) {
                DisplayArmTelemetry();
            }
            armExtension.setPower(0);
        }
        extendOdometer=0;
    }


    /**********************
     * Encoder Methods
     **********************/

    public void StopAndResetAllEncoders() {
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    /**********************
     * Power Methods
     **********************/



    /**********************
     * Telemetry Methods
     **********************/

    public void DisplayArmTelemetry() {
        double extensionInches = armExtension.getCurrentPosition() / ticksPerInchExtension;
        //telemetry.clear();
        //telemetry.addLine()
          //      .addData("Arm Extension ", (int) extensionInches + "   Power: " + "%.1f", armExtension.getPower());
        //telemetry.update();
    }

    //End of Class
}
