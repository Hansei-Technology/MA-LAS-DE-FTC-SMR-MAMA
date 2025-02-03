package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;

@TeleOp(name = "Limelight Test", group = "Tests")
@Config
public class LimelightTest extends LinearOpMode {
    public static int extendoLime = 40;
    public static int timeGlis = 550;
    boolean isretracting = false;
    boolean isDown = false;
    ElapsedTime timer;

    public enum SubmersibleState {
        IDLE,
        EXTENDING,
        TRANSFERING,
        COLLECTING
    }
    SubmersibleState subCS = SubmersibleState.IDLE;

    private Limelight3A limelight;
    private IntakeSubsystem intake;
    private ExtendoSystem extendo;
    private RobotSystems robot;
    private LiftSystem lift;
    private OuttakeSubsystem outtake;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        timer = new ElapsedTime();
        intake = new IntakeSubsystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        robot = new RobotSystems(extendo, lift, intake, outtake);

        limelight.pipelineSwitch(0);
        limelight.start();

        intake.goDown();
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                subCS = SubmersibleState.EXTENDING;
                intake.goDown();
                intake.rotation.goToFlipped();
            }
            if(gamepad1.b) {
                subCS = SubmersibleState.IDLE;
                extendo.goToGround();
            }

            switch (subCS) {
                case IDLE:
                    if(gamepad1.a) {
                        subCS = SubmersibleState.EXTENDING;
                        intake.goDown();
                    }
                    break;
                case EXTENDING:
                    extendo.moveFree(0.3);


                    LLResult result = limelight.getLatestResult();
                    double[] pythonOutput = result.getPythonOutput();
                    int validContours = (int) pythonOutput[0];  // 1 dacă există contururi, 0 altfel
                    double heading = pythonOutput[1] % 180;// Unghiul conturului

                    if(Math.abs(heading - 90) < 5) {
                        heading = 0;
                    } else if(Math.abs (heading - 0) < 5 || Math.abs(heading - 180) < 5) {
                        heading = 90;
                    }

                    if(validContours == 1) {
                        intake.rotation.rotateToAngle((int)heading);
                        extendo.goToPos(extendo.currentPos - extendoLime);
                        subCS = SubmersibleState.COLLECTING;
                        timer.reset();
                    }
                    break;
                case COLLECTING:
                    if(extendo.isAtPosition() || timer.milliseconds() > timeGlis) {
                        intake.collect(true);
                        subCS = SubmersibleState.TRANSFERING;

                    }
                    break;
                case TRANSFERING:

                    break;
            }

            robot.update();
            telemetry.addData("State", subCS);
            telemetry.update();
        }
    }
}
