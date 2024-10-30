package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultDriveSpeed;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PushBot", group ="Test")
//@Disabled
public class Tele_PushBot extends TeleMTZ_Drive_Controls_CS {
    public void runOpMode() throws InterruptedException {

        super.controlRobot("Red", "Center Stage R1", defaultDriveSpeed, true, false, false, true);
    }

}