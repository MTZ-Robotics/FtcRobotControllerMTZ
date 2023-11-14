package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerRevolution1150;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private static final double defaultDriveSpeed = 0.1;
    private static final double defaultTurnSpeed = 0.1;
    private static int defaultPauseTime = 300;

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


    @Override

    public void runOpMode() throws InterruptedException {

        telemetry.log().add("Should Extend 10 Inches, wait 1 second, then retract back at half speed. ");
        waitForStart();

        armExtension = hardwareMap.dcMotor.get("armExtension");
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setDirection(DcMotor.Direction.REVERSE);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armOdometer = 0;
        extendOdometer = 0;

        StopAndResetAllEncoders();
        telemetry.log().clear();
        telemetry.update();
        ExtendArm(10, defaultArmExtensionPower, defaultPauseTime);
        sleep(1000);
        ReturnExtension(); //retract arm
    }

    /**********************
     * Motion Methods
     **********************/


    public void ExtendArm(double additionalExtension, double power,int pause) throws InterruptedException {
        if (opModeIsActive()) {
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
        telemetry.clear();
        telemetry.addLine()
                .addData("Arm Extension ", (int) extensionInches + "   Power: " + "%.1f", armExtension.getPower());
        telemetry.update();
    }

    //End of Class
}
