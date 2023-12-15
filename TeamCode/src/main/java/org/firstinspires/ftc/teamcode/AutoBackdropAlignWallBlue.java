package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Align Wall Blue", group ="A_Top")
//@Disabled
public class AutoBackdropAlignWallBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Backdrop Align Park Wall Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
