package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerRevolution1150;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "GyroDrive", group = "AutoPathsGyroTake2")

public class GyroDrive extends LinearOpMode {

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

    private double currAngle = 0.0;

    /*****************
     * Declare motor & servo objects
     ****************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;

    //double orientationGain = 0.00000000001;
    double orientationGain = 0.00001;
    //double orientationGain = 1*10^-10;
    //double orientationGain = 1*10^-3;

    private static final double ticksPerRevolution = ticksPerRevolution1150;
    private static final double gearReduction = 1.0;
    private static final double wheelDiameterInches = 4.0;
    private static final double pi = 3.1415;
    private static final double conversionTicksToInches = (ticksPerRevolution * gearReduction) / (pi * wheelDiameterInches);

    private static final double driveDistanceAdjustment = .85;


    double startingPosition;


    /***********
     * Main Logic
     */
    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imuForDisplay = hardwareMap.get(BNO055IMU.class, "imu");
        imuForDisplay.initialize(parameters);

        // Map the motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){
            angles=imuForDisplay.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle;
            telemetry.addData("Heading: ",angles.firstAngle);
            telemetry.addData("Roll: ",angles.secondAngle);
            telemetry.addData("Pitch: ",angles.thirdAngle);

            //Display the heading
            telemetry.update();

            //Drive
            driveStraight(40,.1);
            stop();
        }

    }
    public void driveStraight(double inches, double desiredPower){
        /****
         * inches: how far the robot is supposed to travel
         * power: how strong the motors should be targeting
         *
         */
        while(opModeIsActive()) {
            double targetDistance = inches * conversionTicksToInches*driveDistanceAdjustment;
            int i = 0;
            double splitDistance;
            angles = imuForDisplay.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double originalHeading = angles.firstAngle;


            while (targetDistance > 0) {
                if (targetDistance > 50) {
                    //split the target distance up so error can be adjusted
                    if (i < 3) {
                        splitDistance = 10;
                    } else {
                        splitDistance = 50;
                    }
                } else {
                    //use up all of the target distance and go slow
                    splitDistance = (targetDistance);
                }


                //Tell the motors the intended distance
                frontLeft.setTargetPosition((int) (splitDistance));


                //startingPosition = frontLeft.getCurrentPosition();

                // Only drive to the distance for one motor,
                // otherwise the other motors will still want to finish their count
                // and throw the orientation wacky at the end

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //while (frontLeft.getCurrentPosition()<startingPosition+splitDistance) {
                    //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //}

                //Get the current orientation
                angles = imuForDisplay.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                //CCW is Positive for the orientation
                double orientationError = originalHeading - angles.firstAngle;
                //cap the error amount
                if (orientationError>5){
                    orientationError = 5;
                } else if (orientationError<-5) {
                    orientationError = -5;
                }

                //Set the motor power based on the error noted on the imu with gain

                //Subtract Error on Left
                frontLeft.setPower(desiredPower - orientationError * orientationGain);
                backLeft.setPower(desiredPower - orientationError * orientationGain);

                //Add error on right
                frontRight.setPower(desiredPower + orientationError * orientationGain);
                backRight.setPower(desiredPower + orientationError * orientationGain);


                //increase the counters for the next iteration of the loop
                targetDistance = targetDistance - splitDistance;
                while (frontLeft.isBusy()) {
                    telemetry.addData("Target Distance: ", targetDistance);
                    telemetry.addData("Original Heading: ", originalHeading);
                    telemetry.addData("Orientation Error: ", orientationError);
                    telemetry.addData("Orientation Correction: ", orientationError * orientationGain);
                    telemetry.addData("New Right Power: ", desiredPower + orientationError * orientationGain);

                    telemetry.update();
                }
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //wait for humans to read
                sleep(1000);
                i = i + 1;
            }
        }
    }
}

