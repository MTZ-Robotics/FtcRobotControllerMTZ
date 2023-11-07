package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.mtzConstantsPP.defaultDriveSpeed;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Blue Right Strafe", group ="A_Top")
//@Disabled
public class Tele_Blue_Right_Strafe extends TeleMTZ_Drive_Controls_PP {
    public void runOpMode() throws InterruptedException {

        super.controlRobot("Blue", "Freight Frenzy R1", defaultDriveSpeed, true, true, true, true);
    }

}
