package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "GyroDisplay", group = "AutoPathsGyroTake2")

public class GyroDisplay extends LinearOpMode {

    /*********
     * Code adapted from MSET Cuttlefish 6165 tutorial
     */

    /**********
     * State
     */

    // The IMU sensor object
    BNO055IMU imuForDisplay;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    /***********
     * Main Logic
     */
    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imuForDisplay = hardwareMap.get(BNO055IMU.class, "imu");
        imuForDisplay.initialize(parameters);

        waitForStart();

        while (opModeIsActive()){
            angles=imuForDisplay.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading: ",angles.firstAngle);
            telemetry.addData("Roll: ",angles.secondAngle);
            telemetry.addData("Pitch: ",angles.thirdAngle);
            telemetry.update();
        }

    }

}

