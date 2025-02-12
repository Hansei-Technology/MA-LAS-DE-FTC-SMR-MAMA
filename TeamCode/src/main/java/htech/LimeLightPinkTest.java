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

import htech.config.PositionsExtendo;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;

@TeleOp
@Config
public class LimeLightPinkTest extends LinearOpMode {
    public static double extendoMultiplyer = 0.4;
    public static double freeTerm = 0;
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
    private int loopCount=0;


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

        intake.goToWall();

        LLResult result = limelight.getLatestResult();
        double[] pythonoutput = result.getPythonOutput();

        while(opModeInInit()) {
            result = limelight.getLatestResult();
            pythonoutput = result.getPythonOutput();
            telemetry.addData("pythonOutput0", pythonoutput[0]);
            telemetry.addData("pythonOutput1", pythonoutput[1]);
            telemetry.addData("pythonOutput2", pythonoutput[2]);
            telemetry.addData("pythonOutput3", pythonoutput[3]);
            telemetry.update();
        }



        while (opModeIsActive()) {
            if(subCS == SubmersibleState.IDLE) {
                if (pythonoutput[0] == 1) {
                    intake.goDown();
                    extendo.goToPos((int) (freeTerm + PositionsExtendo.max - pythonoutput[2] * extendoMultiplyer));
                    subCS = SubmersibleState.EXTENDING;
                } else {
                    result = limelight.getLatestResult();
                    pythonoutput = result.getPythonOutput();
                }
            }
            telemetry.addData("pythonOutput0", pythonoutput[0]);
            telemetry.addData("pythonOutput1", pythonoutput[1]);
            telemetry.addData("pythonOutput2", pythonoutput[2]);
            telemetry.addData("pythonOutput3", pythonoutput[3]);
            telemetry.update();
            extendo.update();
        }
    }
}
