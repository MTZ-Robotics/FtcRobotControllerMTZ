package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Align Red", group ="A_Top")
//@Disabled
public class AutoAudienceAlignRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Audience Align",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
