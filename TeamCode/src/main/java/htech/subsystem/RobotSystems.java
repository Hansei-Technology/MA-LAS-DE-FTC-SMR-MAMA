package htech.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.config.RobotSettings;

public class RobotSystems {
    public ExtendoSystem extendoSystem;
    public LiftSystem liftSystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    public ElapsedTime timer;
    public ElapsedTime timerCollect;
    public ElapsedTime timerSpecimen;

    public RobotSystems(ExtendoSystem extendoSystem, LiftSystem liftSystem, IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.extendoSystem = extendoSystem;
        this.liftSystem = liftSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new ElapsedTime();
        timerCollect = new ElapsedTime();
        timerSpecimen = new ElapsedTime();
    }

    public void update() {
        extendoSystem.update();
        liftSystem.update();
        intakeSubsystem.update();
        updateTranfer();
        updateCollect();
//        updateHopPeSpate();
    }

    public void startTransfer(boolean sample) {
        transferingSample = sample;
        if(intakeSubsystem.claw.isOpen) {
            intakeSubsystem.claw.close();
            transferState = TransferStates.CLOSING_CLAW;
        } else {
            transferState = TransferStates.LIFT_GOING_DOWN;
        }
    }

    public enum TransferStates {
        IDLE,
        CLOSING_CLAW,
        LIFT_GOING_DOWN,
        INTAKE_DOWN,
        INTAKE_WALL,
        READY_TO_TRANSFER,
        READY_TO_TRANSFER_SAMPLE,
        CATCHING,
        WAITING_TO_CATCH,
        TRANSFER_READY,
        LIFT_GOING_UP,
    }
    public TransferStates transferState = TransferStates.IDLE;

//    public enum HopPeSpateStates {
//        IDLE,
//        CASE1,
//        CASE2,
//        CASE3,
//        CASE4,
//        CASE5
//    }
//    public HopPeSpateStates hopPeSpateState = HopPeSpateStates.IDLE;

    public boolean firstTime = true;
    public boolean transferingSample = true;

    void updateCollect() {
        switch (intakeSubsystem.intakeState) {
            case COLLECT_GOING_DOWN:
                if(firstTime) {
                    timerCollect.reset();
                    firstTime = false;
                }
                if(timerCollect.milliseconds() > RobotSettings.timeToCollectGoingDownFast && intakeSubsystem.fastCollect) {
                    intakeSubsystem.claw.close();
                    outtakeSubsystem.joint.goToTransfer();
                    timerCollect.reset();
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLLECTING;
                }
                if(timerCollect.milliseconds() > RobotSettings.timeToCollectGoingDown && !intakeSubsystem.claw.isOpen) {
                    //intakeSubsystem.claw.close();
                    liftSystem.setPower(-0.3);
                    timerCollect.reset();
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLLECTING;
                }
                break;
            case COLLECTING:
                firstTime = true;
                if(!intakeSubsystem.hopPeSpate && intakeSubsystem.fastCollect && timerCollect.milliseconds() > RobotSettings.timeToCollect) {
                    transferState = TransferStates.LIFT_GOING_DOWN;
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLECT_GOING_UP;
                    timerCollect.reset();
                    intakeSubsystem.fastCollect = false;
                }

                if(intakeSubsystem.hopPeSpate && intakeSubsystem.fastCollect && timerCollect.milliseconds() > RobotSettings.timeToCollect) {
//                    hopPeSpateState = HopPeSpateStates.CASE1;
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.DOWN;
                    timerCollect.reset();
                    intakeSubsystem.fastCollect = false;
                    intakeSubsystem.hopPeSpate = false;
                }

                if(timerCollect.milliseconds() > RobotSettings.timeToCollect) {
                    if(intakeSubsystem.hasElement()) {
                        liftSystem.reset();
                        intakeSubsystem.goToWall();
                        extendoSystem.goToGround();
                    } else {
                        intakeSubsystem.goDownWithoutResetRotation();
                    }
                    timer.reset();
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLECT_GOING_UP;
                }
                break;
            case COLECT_GOING_UP:
                if(!intakeSubsystem.hasElement()) {
                    intakeSubsystem.goDownWithoutResetRotation();
                    extendoSystem.pidEnabled = false;
                }
                if(timer.milliseconds() > RobotSettings.timeToCollectGoingUp) {

                }
                break;
        }
    }


    void updateTranfer() {
        switch (transferState) {
            case IDLE:
                break;

            case CLOSING_CLAW:
                if(timer.milliseconds() > RobotSettings.timeToCloseClaw) {
                    timer.reset();
                    transferState = TransferStates.LIFT_GOING_DOWN;
                }
                break;

            case LIFT_GOING_DOWN:
                //on entry
                liftSystem.goToGround();
                extendoSystem.goToGround();
                if(transferingSample) outtakeSubsystem.goToTransferSample();
                else outtakeSubsystem.goToTransfer();

                outtakeSubsystem.claw.open();
                timer.reset();

                //condition to exit


                if(intakeSubsystem.hasElement()) {
                    if(transferingSample) {
                        intakeSubsystem.goToTransfer(true);
                        timer.reset();
                        transferState = TransferStates.READY_TO_TRANSFER_SAMPLE;
                    } else {
                        if (intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
                            intakeSubsystem.goToReady(transferingSample);
                            transferState = TransferStates.INTAKE_DOWN;
                        } else {
                            intakeSubsystem.goToReady(transferingSample);
                            transferState = TransferStates.INTAKE_WALL;
                        }
                    }
                } else {
                    intakeSubsystem.claw.open();
                    transferState = TransferStates.IDLE;
                }


                break;

            case INTAKE_DOWN:
                if ((liftSystem.isDown() && extendoSystem.isDown() && timer.milliseconds() > RobotSettings.timeDown_Transfer) || timer.milliseconds() > RobotSettings.timeFailedToCloseLift) {
                    intakeSubsystem.goToTransfer(transferingSample);
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case INTAKE_WALL:
                if ((liftSystem.isDown() && extendoSystem.isDown() && timer.milliseconds() > RobotSettings.timeWall_Transfer) || timer.milliseconds() > RobotSettings.timeFailedToCloseLift) {
                    intakeSubsystem.goToTransfer(transferingSample);
                    //intakeSubsystem.claw.goToSliding();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case READY_TO_TRANSFER_SAMPLE:
                if(timer.milliseconds() > RobotSettings.magicTransferTime) intakeSubsystem.claw.goToSliding();


                if (timer.milliseconds() > RobotSettings.timeDown_Transfer_SAMPLE) {
                    timer.reset();
                    outtakeSubsystem.claw.close();
                    transferState = TransferStates.CATCHING;
                }

                break;

            case READY_TO_TRANSFER:
                if (timer.milliseconds() > RobotSettings.timeReady_Transfer) {
                    timer.reset();
                    outtakeSubsystem.claw.close();
                    transferState = TransferStates.CATCHING;
                }
                break;
            case CATCHING:
                if(timer.milliseconds() > RobotSettings.timeToCatch /* && intakeSubsystem.isAtPos()*/) {
                    timer.reset();
                    if(outtakeSubsystem.hasElement()) {
                        intakeSubsystem.claw.open();
                        transferState = TransferStates.WAITING_TO_CATCH;
                    } else {
                        intakeSubsystem.goToWall();
                        transferState = TransferStates.LIFT_GOING_DOWN;
                    }
                }
                break;
            case WAITING_TO_CATCH:
                if (timer.milliseconds() > RobotSettings.timeWaitingToCatch) {
                    timer.reset();
                    intakeSubsystem.goToWall();
                    outtakeSubsystem.goToAfterTransfer();
                    transferState = TransferStates.TRANSFER_READY;
                }
                break;
            case TRANSFER_READY:
                if (timer.milliseconds() > RobotSettings.timeToLastPosTransfer) {
                    timer.reset();
                    transferState = TransferStates.IDLE;
                }
                break;
        }
    }

    public boolean isTransfering() {
        return transferState != TransferStates.IDLE;
    }

//    public void updateHopPeSpate(){
//        switch (hopPeSpateState) {
//            case IDLE:
//                break;
//
//            case CASE1:
//                outtakeSubsystem.joint.catapultarePos();
//                extendoSystem.goToGround();
//                intakeSubsystem.goToReady();
//                hopPeSpateState = HopPeSpateStates.CASE2;
//                timer.reset();
//                break;
//
//            case CASE2:
//                if(timer.milliseconds() > RobotSettings.timeToHopPeSpate) {
//                    intakeSubsystem.claw.open();
//                    hopPeSpateState = HopPeSpateStates.IDLE;
//                }
//                break;
//        }
//
//    }
}
