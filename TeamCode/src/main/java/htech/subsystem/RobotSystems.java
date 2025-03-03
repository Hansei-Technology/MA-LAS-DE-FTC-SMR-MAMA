package htech.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import htech.config.PositionsExtendo;
import htech.config.RobotSettings;

public class RobotSystems {
    public ExtendoSystem extendoSystem;
    public LiftSystem liftSystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    public ElapsedTime timer;
    public ElapsedTime timerCollect;
    public ElapsedTime timerScore;
    public ElapsedTime timerTransfer;

    public boolean transferFirstTime = true;

    public boolean autoSample = false;
    public boolean fastCollect = false;

    public RobotSystems(ExtendoSystem extendoSystem, LiftSystem liftSystem, IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.extendoSystem = extendoSystem;
        this.liftSystem = liftSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new ElapsedTime();
        timerCollect = new ElapsedTime();
        timerScore = new ElapsedTime();
        timerTransfer = new ElapsedTime();
    }

    public void update() {
        extendoSystem.update();
        liftSystem.update();
        intakeSubsystem.update();
        updateTransfer();
        updateCollectSpecimen();
        updateScoreSpecimen();


        if(!extendoSystem.pidEnabled && extendoSystem.currentPos > 150 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) intakeSubsystem.goDownWithoutResetRotation();
        else if(extendoSystem.pidEnabled && extendoSystem.target_position == PositionsExtendo.max && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) intakeSubsystem.goDown();
    }

    public enum TransferStates {
        IDLE,
        LIFT_GOING_DOWN,
        WAITING_FOR_INTAKE_ROTATION,
        EXTENDO_CLOSING,
        OUTTAKE_CLAW_CLOSING,
        INTAKE_CLAW_OPENING,
        GOING_TO_AFTER_TRANSFER
    }

    public TransferStates transferState = TransferStates.IDLE;

    public enum collectSpecimenStates{
        IDLE,
        GETTING_IN_POSITION,
        CLOSING_CLAW,
        WAITING_FOR_CLAW,
        GO_TO_SCORE,
    }
    public collectSpecimenStates collectSpecimenState = collectSpecimenStates.IDLE;

    public enum scoreSpecimenStates{
        IDLE,
        GO_TO_SCORE,
        WAITING_TO_SCORE,
        OPEN_CLAW
    }
    public scoreSpecimenStates scoreSpecimenState = scoreSpecimenStates.IDLE;



    public void transfer() {
        transferState = TransferStates.LIFT_GOING_DOWN;
        transferFirstTime = true;
        timerTransfer.reset();
    }

    public void updateTransfer() {
        switch (transferState) {
            case LIFT_GOING_DOWN:
                if(transferFirstTime) {
                    liftSystem.goToGround();
                    outtakeSubsystem.goToTransfer();
                    intakeSubsystem.goToTransfer();
                    transferFirstTime = false;
                }
                if(liftSystem.isDown() && timerTransfer.milliseconds() > RobotSettings.outtake_going_to_transfer) {
                    if(intakeSubsystem.rotation.rotLevel > 3) {
                        transferState = TransferStates.WAITING_FOR_INTAKE_ROTATION;
                    } else {
                        transferState = TransferStates.EXTENDO_CLOSING;
                    }
                    transferFirstTime = true;
                    timerTransfer.reset();
                }
                break;
            case WAITING_FOR_INTAKE_ROTATION:
                if(timerTransfer.milliseconds() > RobotSettings.rotation_max_time) {
                    transferState = TransferStates.EXTENDO_CLOSING;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case EXTENDO_CLOSING:
                if(transferFirstTime) {
                    extendoSystem.goToTransfer();
                    transferFirstTime = false;
                }
                if(extendoSystem.isAtPosition()) {
                    transferState = TransferStates.OUTTAKE_CLAW_CLOSING;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case OUTTAKE_CLAW_CLOSING:
                if(transferFirstTime) {
                    outtakeSubsystem.claw.close();
                    transferFirstTime = false;
                }
                if(timerTransfer.milliseconds() > RobotSettings.outtake_claw_close) {
                    transferState = TransferStates.INTAKE_CLAW_OPENING;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case INTAKE_CLAW_OPENING:
                if(transferFirstTime) {
                    intakeSubsystem.claw.open();
                    transferFirstTime = false;
                }
                if(timerTransfer.milliseconds() > RobotSettings.intake_claw_open) {
                    transferState = TransferStates.GOING_TO_AFTER_TRANSFER;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case GOING_TO_AFTER_TRANSFER:
                if(transferFirstTime) {
                    outtakeSubsystem.goToAfterTransfer();
                    intakeSubsystem.goToReady();
                    transferFirstTime = false;
                }
                if(timerTransfer.milliseconds() > RobotSettings.going_after_transfer) {
                    transferState = TransferStates.IDLE;
                }
                break;
        }
    }


    public boolean isTransfering(){
        return transferState != TransferStates.IDLE;
    }

    public void updateCollectSpecimen(){

        switch (collectSpecimenState){

            case IDLE:
                break;

            case GETTING_IN_POSITION:
                liftSystem.goToGround();
                outtakeSubsystem.goToCollectSpecimen();
                collectSpecimenState = collectSpecimenStates.CLOSING_CLAW;
                break;

            case CLOSING_CLAW:
                if(!outtakeSubsystem.claw.isOpen){
                    collectSpecimenState = collectSpecimenStates.WAITING_FOR_CLAW;
                    timerCollect.reset();
                }
                break;

            case WAITING_FOR_CLAW:
                if(timerCollect.milliseconds() > 100){
                    collectSpecimenState = collectSpecimenStates.GO_TO_SCORE;
                }
                break;

            case GO_TO_SCORE:
                liftSystem.goToHighChamber();
                outtakeSubsystem.goToSpecimenPrescore();
                break;
        }

    }

    public void collectSpecimen(){
        collectSpecimenState = collectSpecimenStates.GETTING_IN_POSITION;
    }

    public void updateScoreSpecimen(){

        switch (scoreSpecimenState){

            case IDLE:
                break;

            case GO_TO_SCORE:
                outtakeSubsystem.goToSpecimenScore();
                scoreSpecimenState = scoreSpecimenStates.WAITING_TO_SCORE;
                timerScore.reset();
                break;

            case WAITING_TO_SCORE:
                if(timerScore.milliseconds() > 200){
                    scoreSpecimenState = scoreSpecimenStates.OPEN_CLAW;
                }
                break;

            case OPEN_CLAW:
                outtakeSubsystem.claw.open();
                scoreSpecimenState = scoreSpecimenStates.IDLE;
                break;
        }

    }

    public void scoreSpecimen(){
        scoreSpecimenState = scoreSpecimenStates.GO_TO_SCORE;
    }



}
