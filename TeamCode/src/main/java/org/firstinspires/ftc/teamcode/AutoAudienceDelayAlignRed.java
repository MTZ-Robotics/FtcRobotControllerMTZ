package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Delay Align Red", group ="A_Top")
//@Disabled
public class AutoAudienceDelayAlignRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Audience Early Delay Align Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
