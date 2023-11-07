package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="RightSampleClose", group ="A_Top")
//@Disabled
public class AutoRightSampleClose extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","RightBlueShortConePark",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
