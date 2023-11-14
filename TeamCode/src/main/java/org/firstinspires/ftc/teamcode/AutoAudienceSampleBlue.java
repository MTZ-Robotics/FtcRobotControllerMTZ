package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Sample Blue", group ="A_Top")
//@Disabled
public class AutoAudienceSampleBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Audience Sample",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
