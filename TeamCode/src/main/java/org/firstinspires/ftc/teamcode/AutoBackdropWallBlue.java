package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Wall Blue", group ="A_Top")
//@Disabled
public class AutoBackdropWallBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Backdrop Wall",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
