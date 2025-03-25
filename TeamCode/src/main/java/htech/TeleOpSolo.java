package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.classes.StickyGamepad;
import htech.config.PositionsExtendo;
import htech.config.PositionsLift;
import htech.subsystem.ChassisMovement;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.HangSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class TeleOpSolo extends LinearOpMode {

    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    RobotSystems robotSystems;
    HangSystem hang;

    boolean firstTime = true;
    boolean hasElement = false;


    @Override
    public void runOpMode() throws InterruptedException {
        hang = new HangSystem(hardwareMap);
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);
        waitForStart();

        intakeSubsystem.init();
        outtakeSubsystem.init();

        while (opModeIsActive()){

            if(gamepad1.right_trigger > 0.1){
                chassisMovement.updateMovement(gamepad1);
            }
            else{
                chassisMovement.updateMovementSlowRotation(gamepad1);
            }

            if(stickyGamepad1.right_bumper){
                if(extendo.target_position != PositionsExtendo.max){
                    extendo.goToMax();
                    intakeSubsystem.goDown();
                    robotSystems.transferState = RobotSystems.TransferStates.IDLE;
                }
                else{
                    intakeSubsystem.collect();
                    robotSystems.timer.reset();
                }
            }

            if(stickyGamepad1.right_stick_button){
                intakeSubsystem.rotation.togglePerpendicular();
            }

            if(stickyGamepad1.left_bumper){
                if(lift.target_position == PositionsLift.ground){
                    lift.goToHighBasket();
                    outtakeSubsystem.goToSampleScore();
                    hasElement = true;
                }
                else if(lift.target_position == PositionsLift.highBasket){
                    if(hasElement){
                        outtakeSubsystem.claw.open();
                        hasElement = false;
                    }
                    else{
                        lift.goToGround();
                        outtakeSubsystem.goToTransfer();
                    }
                }
            }

            if(gamepad1.dpad_down) {
                while (gamepad1.dpad_down) {
                    lift.setPower(-0.5);
                }
                lift.setPower(0);
                lift.reset();
                gamepad1.rumble(100);
            }

            if(gamepad1.dpad_up){
                while(gamepad1.dpad_up){
                    extendo.setPower(-0.35);
                }
                extendo.setPower(0);
                extendo.reset();
                gamepad1.rumble(100);
            }

            telemetry.addData("[STATUS]", "Main Teleop is running.");
            telemetry.addData("Match Time", matchTimer.seconds());
            telemetry.addData("Lift", lift.currentPos);
            telemetry.addData("lift is at POS", lift.isAtPosition());
            telemetry.addData("Extendo", extendo.currentPos);
            telemetry.addData("Intake", intakeSubsystem.intakeState);
            telemetry.addData("intakeTimer", robotSystems.timerCollect.milliseconds());
            telemetry.addData("extendoPID", extendo.pidEnabled);
            telemetry.addData("intakeRot", intakeSubsystem.rotation.rotLevel);
            telemetry.addData("NO COLOR", robotSystems.noColor);

            robotSystems.update();
            telemetry.update();
            stickyGamepad1.update();
        }
    }
}
