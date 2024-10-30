package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mtzConstantsCS.defaultDriveSpeed;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="pushbot2", group ="A_Top")
//@Disabled
public class pushbot2 extends TeleMTZ_Drive_Controls_CS {
    public void runOpMode() throws InterruptedException {

        super.controlRobot("Red", "Center Stage R1", defaultDriveSpeed, true, false, false, true);
    }

}
