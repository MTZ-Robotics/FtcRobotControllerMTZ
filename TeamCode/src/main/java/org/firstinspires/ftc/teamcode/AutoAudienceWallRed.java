package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Wall Red", group ="A_Top")
//@Disabled
public class AutoAudienceWallRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Audience Wall",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
