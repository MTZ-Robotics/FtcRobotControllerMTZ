package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Align Blue", group ="A_Top")
//@Disabled
public class AutoAudienceAlignBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Audience Align",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
