package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Delay Red", group ="A_Top")
//@Disabled
public class AutoAudienceDelayRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Audience Early Delay Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
