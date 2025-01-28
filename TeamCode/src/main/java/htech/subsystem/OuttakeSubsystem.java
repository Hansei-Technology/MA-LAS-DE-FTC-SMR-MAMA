package htech.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.mechanism.intake.IntakeBar;
import htech.mechanism.outtake.LimitSwitch;
import htech.mechanism.outtake.OuttakeClaw;
import htech.mechanism.outtake.OuttakeJoint;
@Config
public class OuttakeSubsystem {
    public final OuttakeClaw claw;
    public final OuttakeJoint joint;
    public final LimitSwitch limitSwitch;

    public enum outtakeStates {
        SPECIMEN,
        BASKET,
        TRANSFER
    }
    public outtakeStates CS = outtakeStates.TRANSFER;
    //timers are in milliseconds
    public static int timeToMove = 100;


    public OuttakeSubsystem(HardwareMap hardwareMap) {
        limitSwitch = new LimitSwitch(hardwareMap);
        // MECHANISM //
        claw = new OuttakeClaw(hardwareMap);
        joint = new OuttakeJoint(hardwareMap);
    }

    public void init() {
        joint.goToTransfer();
        claw.open();
    }

    public void goToTransferSample() {
        joint.goToTransferSample();
        claw.open();
    }

    public void goToTransfer() {
        joint.goToTransfer();
        claw.open();
    }

    public boolean hasElement() {
        return limitSwitch.isPressed();
    }

    public void goToSampleScore() {
        joint.goToBasketScore();
    }

    public void goToSpecimenScore() {
        joint.goToSpecimenScore();
    }

    public void goToAfterTransfer() {
        joint.goToAfterTransfer();
    }
}
