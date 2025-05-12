package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class LimelightTest extends LinearOpMode {

    Limelight3A ll;
    LLResult result;

    public static double h = 9;

    @Override
    public void runOpMode() throws InterruptedException {
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ll.start();


        waitForStart();

        while (opModeIsActive()){

            double y = h * Math.tan(60 - result.getTy());
            double x = 2 * h * Math.tan(result.getTx());

            telemetry.addData("x", x);
            telemetry.addData("y", y);

            result =  ll.getLatestResult();
        }
    }
}
