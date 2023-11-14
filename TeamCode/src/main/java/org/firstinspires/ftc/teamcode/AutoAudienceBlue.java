package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Blue", group ="A_Top")
//@Disabled
public class AutoAudienceBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Audience",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
