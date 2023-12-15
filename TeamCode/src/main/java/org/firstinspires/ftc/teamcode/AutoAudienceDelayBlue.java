package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Delay Blue", group ="A_Top")
//@Disabled
public class AutoAudienceDelayBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Audience Early Delay Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
