package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="LucyCode", group ="A_Top")
//@Disabled
public class AutoLucyCode extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Lucy Code",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
