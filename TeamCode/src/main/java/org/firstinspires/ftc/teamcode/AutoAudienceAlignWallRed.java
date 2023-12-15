package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Align Wall Red", group ="A_Top")
//@Disabled
public class AutoAudienceAlignWallRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Audience Align Park Wall Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
