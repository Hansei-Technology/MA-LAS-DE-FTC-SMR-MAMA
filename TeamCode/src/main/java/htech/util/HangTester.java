package htech.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import htech.subsystem.ChassisMovement;
import htech.subsystem.HangSystem;

@TeleOp (name = "[UTIL] Hang Tester", group = "HTech")
public class HangTester extends LinearOpMode {
    HangSystem hang;
    ChassisMovement chassis;
    @Override
    public void runOpMode() throws InterruptedException {
        hang = new HangSystem(hardwareMap);
        chassis = new ChassisMovement(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {

            hang.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            chassis.hangChassis(gamepad1);

            telemetry.addData("[STATUS]", "HangTester Teleop is running.");
            telemetry.update();
        }
    }
}
