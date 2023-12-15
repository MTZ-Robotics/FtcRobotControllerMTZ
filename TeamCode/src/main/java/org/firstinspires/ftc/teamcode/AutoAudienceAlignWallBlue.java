package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Align Wall Blue", group ="A_Top")
//@Disabled
public class AutoAudienceAlignWallBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Audience Align Park Wall Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
