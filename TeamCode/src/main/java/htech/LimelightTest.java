package htech;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;

@TeleOp(name = "Limelight Test", group = "Tests")
@Config
public class LimelightTest extends LinearOpMode {
    public static int extendoLime = 100;

    private Limelight3A limelight;
    private IntakeSubsystem intake;
    private ExtendoSystem extendo;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intake = new IntakeSubsystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);

        limelight.pipelineSwitch(0);
        limelight.start();

        intake.goDown();
        waitForStart();

        while (opModeIsActive()) {

            extendo.moveFree(gamepad1.right_trigger - gamepad1.left_trigger);


            LLResult result = limelight.getLatestResult();
            double[] pythonOutput = result.getPythonOutput();
            int validContours = (int) pythonOutput[0];  // 1 dacă există contururi, 0 altfel
            double heading = pythonOutput[1];// Unghiul conturului

            telemetry.addData("opModeIsActive", opModeIsActive());
            telemetry.addData("validContours", validContours);
            telemetry.addData("heading", heading);


            if(validContours == 1) {
                extendo.goToPos(extendo.currentPos - extendoLime);
                intake.rotation.rotateToAngle((int)heading);
            }
            //telemetry.addData("distance", distance);
            telemetry.update();
        }
    }
}
