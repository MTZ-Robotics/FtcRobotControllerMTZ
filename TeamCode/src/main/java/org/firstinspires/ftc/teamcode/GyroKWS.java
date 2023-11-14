package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.driveEfficiency;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerInchWheelDrive;
import static org.firstinspires.ftc.teamcode.mtzConstantsCS.ticksPerRevolution1150;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "GyroTestingKWS", group = "AutoPathsGyroTake2")
    public class GyroKWS extends LinearOpMode {

    /**********
     * State
     */

    // The IMU sensor object
    BNO055IMU imuForError;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

/*  Commented Out to use current example
    BNO055IMU imu;
    //HardwareMap robot = new HardwareMap();
        private ElapsedTime runtime = new ElapsedTime();



*/

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    /*****************
     * Declare motor & servo objects
     ****************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;


    private static final double ticksPerRevolution = ticksPerRevolution1150;

    /***********
     * Main Logic
     */
    @Override
        public void runOpMode() {


            //imu.initialize(hardwareMap.getParameters());
            waitForStart();
            driveStraight(20.5,50);
        }
public void driveStraight(double inches, double power){
    /****
     * inches: how far the robot is supposed to travel
     * power: how strong the motors should be targeting
     *
     * Theory:
     *      If the robot starts heading in a different direction than it started, adjust
     *      the motor power on each of the motors to account for the error.
     *
     *      Target angle = initial angle
     *
     *      error adjustment should be reset to zero when the high level opmode is run
     *
     *      motor power (each) = power - error adjustment
     *
     *      error adjustment should be based on a turn maneuver
     *
     *      target distance starts at inches and reduces each time around the loop
     *      based on motor encoders
     *
     *      Initial loop iterations should have small distance to have the ability to adjust error
     *
     *      Final loop iterations should have small distance and reduced power to narrow in
     *      on target distance
     *
     *
     */
     double targetDistance = (inches*(driveEfficiency));
     int i=0;
     double splitDistance;
     double motorPower[] = {power,power,power,power};
     while(targetDistance>0){
         if(targetDistance>2){
             //split the target distance up so error can be adjusted
             if(i<3) {
                 splitDistance = 1;
             } else {
                 splitDistance = targetDistance/3;
             }
         } else {
             //use up all of the target distance and go slow
             splitDistance = targetDistance;
             // Reduce the motor speeds on each motor
             for (int j = 0; j < 3; j++) {
                 motorPower[j] =.2*motorPower[j];
             }
         }

         //Set the distance that the motor should travel
         frontLeft.setTargetPosition((int) (splitDistance * ticksPerInchWheelDrive));
         frontRight.setTargetPosition((int) (splitDistance * ticksPerInchWheelDrive));
         backLeft.setTargetPosition((int) (-1 * splitDistance * ticksPerInchWheelDrive));
         backRight.setTargetPosition((int) (-1 * splitDistance * ticksPerInchWheelDrive));

         //Set the motor power based on the error noted on the imu


         //Tell the motor to go


         //increase the counters for the next iteration of the loop
         targetDistance = targetDistance-splitDistance;
         i=i+1;
     }
    //
}

    /*
     * Code from examples that isn't working

        public void resetAngle() {

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, BNO055IMU.AngleUnit.DEGREES);
            currAngle = 0;
        }

        public double getAngle() {
            Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, BNO055IMU.AngleUnit.DEGREES);
            double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

            if (deltaAngle > 180) {
                deltaAngle -= 360;
            } else if (deltaAngle <= -180) {
                deltaAngle += 360;
            }
            currAngle += deltaAngle;
            lastAngles = orientation;
            telemetry.addData("gyro", orientation.firstAngle);
            return currAngle;
        }

        public void turn(double degrees) {
            resetAngle();
            double error = degrees;
            while (opModeIsActive() && Math.abs(error) > 2) {
                double motorPower = (error < 0 ? -0.3 : 0.3);
                DcMotor.setPower(-motorPower, motorPower, -motorPower, motorPower);
                error = degrees - getAngle();
                telemetry.addData("error", error);
                telemetry.update();
            }
           // robot.setPower(0);
            DcMotor.setPower(0);
        }

        public void turnTo(double degrees) {
            Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, BNO055IMU.AngleUnit.DEGREES);
            robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, BNO055IMU.AngleUnit.DEGREES);
            double error = degrees - orientation.firstAngle;

            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }
        }     */


    }

