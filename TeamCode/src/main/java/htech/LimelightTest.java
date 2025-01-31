package htech;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight Test", group = "Tests")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double[] pythonOutput = result.getPythonOutput();
            int validContours = (int) pythonOutput[0];  // 1 dacă există contururi, 0 altfel
            double cx = pythonOutput[1];                // Coordonata X a conturului
            double cy = pythonOutput[2];                // Coordonata Y a conturului
            double heading = pythonOutput[3];           // Unghiul conturului
            double distance = pythonOutput[4];
            telemetry.addData("opModeIsActive", opModeIsActive());
            telemetry.addData("validContours", validContours);
            telemetry.addData("cx", cx);
            telemetry.addData("cy", cy);
            telemetry.addData("heading", heading);
            telemetry.addData("distance", distance);
            telemetry.update();
        }
    }
}
