package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Auto Calibrate", group ="z_test")
//@Disabled
public class z_AutoCalibrate extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            super.autoPaths("Blue","Calibrate",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
