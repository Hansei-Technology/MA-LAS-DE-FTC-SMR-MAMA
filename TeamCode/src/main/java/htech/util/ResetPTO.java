package htech.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import htech.mechanism.hang.PTO;

@TeleOp(name = "[UTIL] RESET PTO", group = "HTECH")
public class ResetPTO extends LinearOpMode {

    PTO pto;

    @Override
    public void runOpMode() throws InterruptedException {
        pto = new PTO(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_right){
                pto.resetRight();
            }
            if(gamepad1.dpad_left){
                pto.resetLeft();
            }
        }
    }
}
