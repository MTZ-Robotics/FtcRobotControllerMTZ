package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Red", group ="A_Top")
//@Disabled
public class AutoAudienceRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Audience",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
