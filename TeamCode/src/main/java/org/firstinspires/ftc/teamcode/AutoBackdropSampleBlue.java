package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Sample Blue", group ="A_Top")
//@Disabled
public class AutoBackdropSampleBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Backdrop Sample",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
