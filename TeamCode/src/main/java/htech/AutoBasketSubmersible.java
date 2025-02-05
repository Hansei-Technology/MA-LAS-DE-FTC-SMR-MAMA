package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import htech.config.PositionsExtendo;
import htech.subsystem.ChassisMovement;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous (name = "sampleFromSUbmersible", group = "HTECH")
public class AutoBasketSubmersible extends OpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    Limelight3A limelight;


    private Telemetry telemetryA;

    private Follower follower;

    private Path goToPreload;

    public static int timeToPlacePreload = 0;


    public static double fastSpeed = 1;
    public static double slowSpeed = 1;

    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = -21, PRELOAD_Y = 9, PRELOAD_ANGLE = 45;
    //public static double SAFE_X = -13, SAFE_Y = -20, SAFE_ANGLE;
    //public static double SAFE_BASKET_X = -20, SAFE_BASKET_Y = -10, SAFE_BASKET_ANGLE;
    public static double SAMPLE1_X = -18.3, SAMPLE1_Y = 15.3, SAMPLE1_ANGLE = 90;
    public static double SAMPLE2_X = -26.3, SAMPLE2_Y = 22, SAMPLE2_ANGLE = 90;
    public static double SAMPLE3_X = -27.4, SAMPLE3_Y = 20.2, SAMPLE3_ANGLE = 115;
    public static double BASKET1_X = -21.5, BASKET1_Y = 9.5, BASKET1_ANGLE = 45;
    public static double BASKET2_X = -21.5, BASKET2_Y = 9.5, BASKET2_ANGLE = 45;
    public static double BASKET3_X = -21, BASKET3_Y = 9.5, BASKET3_ANGLE = 45;
//    public static double PARK_X = 7.5, PARK_Y = 60, PARK_ANGLE = 180;
//    public static double SAFE_PARK_X = -10, SAFE_PARK_Y = 52, SAFE_PARK_ANGLE;
    public static double SAFE_X = -22, SAFE_Y = 60, SAFE_ANGLE;
    public static double SUBMERSIBLE_X = 9, SUBMERSIBLE_Y = 52, SUBMERSIBLE_ANGLE = 0;
    public static double SUBMERSIBLE2_X = 9, SUBMERSIBLE2_Y = 52, SUBMERSIBLE2_ANGLE = 0;

    public static double liftMagic = 1300;
    boolean offf = false;

    public static int timeToPreload = 0;
    public static int timeToSample = 0;
    public static int timeToCollect1 = 700;
    public static int timeToCollect2 = 600;
    public static int timeToCollect3 = 700;
    public static int time_to_transfer = 1000;
    public static int time_to_lift = 580;
    public static int time_to_drop = 650;
    public static int timeGlis = 500;
    public static int timeToCheckCollect = 400;
    public static int timeTryingToCollect = 1500;


    public static double ValueTForLowerLift1 = 0.55;
    public static double ValueTForLowerLift2 = 0.65;
    public static double ValueTForLowerLift3 = 0.65;
    public static double ValueTForLowerLiftSub = 0.07;


    public static int extendoPoz1 = 330;
    public static int extendoPoz2 = 160;
    public static int extendoPoz3 = 280;

    Path goTo1Sample;
    Path goTo2Sample;
    Path goTo3Sample;
    Path goTo1Basket;
    Path goTo2Basket;
    Path goTo3Basket;
    Path goToSubmersible;
    Path goToSubmersible2;
    Path goToSample2From1;
    Path goTosample3From2;
    Path goToBasketFromSub;


    LLResult result;
    double[] pythonOutput;
    int validContours;
    double heading;

    private enum STATES {
        MOVING,
        WAITING,
        TRANSFERING,
        PRELOAD,
        PLACING_PRELOAD,
        PLACING_PRELOAD_2,
        SAMPLE1,
        SAMPLE2,
        SAMPLE3,
        BASKET1,
        BASKET2,
        BASKET3,
        COLLECTING1,
        COLLECTING2,
        COLLECTING3,
        GO_SUBMERSIBLE,
        COLLECTING_SUBMERSIBLE,
        COLLECTING_SUBMERSIBLE2,
        CHECK_COLLECT_SUBMERSIBLE,
        GO_TO_BASKET_FROM_SUB
    }
    public STATES CS = STATES.PRELOAD, NS = STATES.MOVING;
    public int TIME_TO_WAIT = 0;

    Pose Sample1 = new Pose(SAMPLE1_X, SAMPLE1_Y, SAMPLE1_ANGLE);
    Pose Sample2 = new Pose(SAMPLE2_X, SAMPLE2_Y, SAMPLE2_ANGLE);
    Pose Sample3 = new Pose(SAMPLE3_X, SAMPLE3_Y, SAMPLE3_ANGLE);
    Pose Basket1 = new Pose(BASKET1_X, BASKET1_Y, BASKET1_ANGLE);
    Pose Basket2 = new Pose(BASKET2_X, BASKET2_Y, BASKET2_ANGLE);
    Pose Basket3 = new Pose(BASKET3_X, BASKET3_Y, BASKET3_ANGLE);

    private int loopCount = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        robotSystems.autoSample = true;
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        intakeSubsystem.initAuto();
        outtakeSubsystem.init();

        outtakeSubsystem.claw.close();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, START_ANGLE));

        goToPreload = new Path(
                // Line 1
                new BezierLine(
                        new Point(START_X,START_Y, Point.CARTESIAN),
                        new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN)
                )
        );
        goToPreload.setLinearHeadingInterpolation(Math.toRadians(START_ANGLE), Math.toRadians(PRELOAD_ANGLE));


        goTo1Sample = new Path(
                // Line 1
                new BezierLine(
                        new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN),
                        new Point(SAMPLE1_X,SAMPLE1_Y, Point.CARTESIAN)
                )
        );
        goTo1Sample.setLinearHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE), Math.toRadians(SAMPLE1_ANGLE));



        goTo1Basket = new Path(
                new BezierLine(
                        new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                        new Point(BASKET1_X, BASKET1_Y, Point.CARTESIAN)
                )
        );
        goTo1Basket.setLinearHeadingInterpolation(Math.toRadians(SAMPLE1_ANGLE), Math.toRadians(BASKET1_ANGLE));


        goTo2Sample = new Path(
                // Line 1
                new BezierLine(
                        new Point(BASKET1_X,BASKET1_Y, Point.CARTESIAN),
                        new Point(SAMPLE2_X,SAMPLE2_Y, Point.CARTESIAN)
                )
        );
        goTo2Sample.setLinearHeadingInterpolation(Math.toRadians(BASKET1_ANGLE), Math.toRadians(SAMPLE2_ANGLE));


        goTo2Basket = new Path(
                new BezierLine(
                        new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                        new Point(BASKET2_X, BASKET2_Y, Point.CARTESIAN)
                )
        );
        goTo2Basket.setLinearHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE), Math.toRadians(BASKET2_ANGLE));



        goTo3Sample = new Path(
                // Line 1
                new BezierLine(
                        new Point(BASKET2_X,BASKET2_Y, Point.CARTESIAN),
                        new Point(SAMPLE3_X,SAMPLE3_Y, Point.CARTESIAN)
                )
        );
        goTo3Sample.setLinearHeadingInterpolation(Math.toRadians(BASKET2_ANGLE), Math.toRadians(SAMPLE3_ANGLE));


        goTo3Basket = new Path(
                new BezierLine(
                        new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
                        new Point(BASKET3_X, BASKET3_Y, Point.CARTESIAN)
                )
        );
        goTo3Basket.setLinearHeadingInterpolation(Math.toRadians(SAMPLE3_ANGLE), Math.toRadians(BASKET3_ANGLE));

//        goToPark = new Path(
//                new BezierCurve(
//                        new Point(BASKET3_X, BASKET3_Y, Point.CARTESIAN),
//                        new Point(SAFE_PARK_X, SAFE_PARK_Y, Point.CARTESIAN),
//                        new Point(PARK_X, PARK_Y, Point.CARTESIAN)
//                )
//        );
//        goToPark.setLinearHeadingInterpolation(Math.toRadians(BASKET3_ANGLE), Math.toRadians(PARK_ANGLE));

        goToSubmersible = new Path(
                new BezierCurve(
                        new Point(BASKET3_X, BASKET3_Y, Point.CARTESIAN),
                        new Point(SAFE_X, SAFE_Y, Point.CARTESIAN),
                        new Point(SUBMERSIBLE_X, SUBMERSIBLE_Y, Point.CARTESIAN)
                )
        );
        goToSubmersible.setLinearHeadingInterpolation(Math.toRadians(BASKET3_ANGLE), Math.toRadians(SUBMERSIBLE_ANGLE));

        goTosample3From2 = new Path(
                new BezierLine(
                        new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                        new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN)
                )
        );
        goTosample3From2.setLinearHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE), Math.toRadians(SAMPLE3_ANGLE));

        goToSample2From1 = new Path(
                new BezierLine(
                        new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                        new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN)
                )
        );
        goToSample2From1.setLinearHeadingInterpolation(Math.toRadians(SAMPLE1_ANGLE), Math.toRadians(SAMPLE2_ANGLE));

        follower.setMaxPower(slowSpeed);
        outtakeSubsystem.goToSampleScore();
        lift.goToHighBasket();
        follower.followPath(goToPreload);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    boolean firstTime = true;

    @Override
    public void loop() {
        follower.update();
        robotSystems.update();


        switch (CS) {
            case PRELOAD:
                //lift.goToHighBasket();
                //outtakeSubsystem.goToSampleScore();
                NS = STATES.PLACING_PRELOAD;
                CS = STATES.MOVING;
                break;

            case MOVING:

                if(NS == STATES.COLLECTING1) {
                    if(follower.getCurrentTValue() > ValueTForLowerLift1) {
                        lift.goToGround();
                        outtakeSubsystem.goToTransfer();
                    }
                }

                if(NS == STATES.COLLECTING2) {
                    if(follower.getCurrentTValue() > ValueTForLowerLift2) {
                        lift.goToGround();
                        outtakeSubsystem.goToTransfer();
                    }
                }

                if(NS == STATES.COLLECTING3) {
                    if(follower.getCurrentTValue() > ValueTForLowerLift3) {
                        lift.goToGround();
                        outtakeSubsystem.goToTransfer();
                    }
                }

                if(NS == STATES.COLLECTING_SUBMERSIBLE) {
                    if(follower.getCurrentTValue() > ValueTForLowerLiftSub) {
                        lift.goToGround();
                        outtakeSubsystem.goToTransfer();
                    }
                }

                if(NS == STATES.COLLECTING_SUBMERSIBLE) {
                    extendo.goToPos(45);
                    if(follower.getCurrentTValue() > 0.99 && extendo.isAtPosition()) {
                        firstTime = true;
                        timer.reset();
                        CS = NS;
                    }
                } else if(!follower.isBusy() ||  follower.getCurrentTValue() > 0.99) {
                    firstTime = true;
                    timer.reset();
                    CS = NS;
                }
                break;

            case WAITING:
                if(timer.milliseconds() > TIME_TO_WAIT) {
                    CS = NS;
                }
                break;

            case TRANSFERING:
                if(robotSystems.transferState == RobotSystems.TransferStates.IDLE) {
                        lift.goToHighBasket();
                        firstTime = true;
                        CS = STATES.MOVING;
                }
                break;

            case PLACING_PRELOAD:
//                if(firstTime) {
//                    lift.goToHighBasket();
//                    firstTime = false;
//                }

                if(lift.isAtPosition() || timer.milliseconds() > timeToPlacePreload) {
                    outtakeSubsystem.claw.open();
                    firstTime = false;
                    NS = STATES.PLACING_PRELOAD_2;
                    CS = STATES.WAITING;
                    TIME_TO_WAIT = timeToPreload;
                    timer.reset();
                }


                break;

            case PLACING_PRELOAD_2:
//                outtakeSubsystem.goToTransfer();
//                lift.goToGround();
                intakeSubsystem.goDown();
                firstTime = true;
                //follower.setMaxPower(fastSpeed);
                //follower.followPath(goTo1Sample, true);
                CS = STATES.SAMPLE1;


//                if(lift.isDown()) {
//                    CS = STATES.SAMPLE1;
//                    //CS = STATES.MOVING;
//                    firstTime = true;
//                }

                break;

            case SAMPLE1:
                timer.reset();
                follower.setMaxPower(fastSpeed);
                follower.followPath(goTo1Sample, true);
                intakeSubsystem.goDown();
                intakeSubsystem.claw.open();
                timer.reset();
                NS = STATES.COLLECTING1;
                CS = STATES.MOVING;
                break;

            case COLLECTING1:
                if(firstTime) {
                    extendo.goToPos(extendoPoz1);
                    timer.reset();
                    firstTime = false;
                }
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.COLECT_GOING_UP || intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) {
                    if(intakeSubsystem.hasElement()) {
                        follower.setMaxPower(fastSpeed);
                        follower.followPath(goTo1Basket, true);
                        //robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN; //start transfer
                        NS = STATES.BASKET1;
                        firstTime = true;
                        CS = STATES.TRANSFERING;
                    } else {
                        follower.setMaxPower(slowSpeed);
                        follower.followPath(goToSample2From1, true);

                        intakeSubsystem.goDown();
                        intakeSubsystem.rotation.goToFlipped();
                        intakeSubsystem.claw.open();

                        timer.reset();
                        NS = STATES.COLLECTING2;
                        CS = STATES.MOVING;
                    }

                }
                if(timer.milliseconds() > timeToCollect1 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.collect(true);
                    timer.reset();
                }
                break;

            case BASKET1:
                //lift.goToHighBasket();
                if(firstTime) {
                    offf = false;
                    timer.reset();
                    firstTime = false;
                }

                if(timer.milliseconds() > time_to_lift) {
                    outtakeSubsystem.goToSampleScore();
                }

                if(timer.milliseconds() > time_to_drop) {
                    outtakeSubsystem.claw.open();
                    timer.reset();
                    TIME_TO_WAIT = timeToSample;
                    NS = STATES.SAMPLE2;
                    CS = STATES.WAITING;
                }
                break;

            case SAMPLE2:
                follower.setMaxPower(fastSpeed);
                follower.followPath(goTo2Sample, true);
                //outtakeSubsystem.goToTransfer();
                //lift.goToGround();
                intakeSubsystem.goDown();
                intakeSubsystem.rotation.goToFlipped();
                intakeSubsystem.claw.open();

                timer.reset();
                NS = STATES.COLLECTING2;
                CS = STATES.MOVING;
                break;

            case COLLECTING2:
                if(firstTime) {
                    timer.reset();
                    //outtakeSubsystem.goToTransfer();
                    extendo.goToPos(extendoPoz2);
                    firstTime = false;
                }
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.COLECT_GOING_UP || intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) {
                    if(intakeSubsystem.hasElement()) {
                        follower.setMaxPower(fastSpeed);
                        follower.followPath(goTo2Basket, true);
                        //robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN; //start transfer
                        NS = STATES.BASKET2;
                        firstTime = true;
                        CS = STATES.TRANSFERING;
                    } else {
                        follower.setMaxPower(slowSpeed);
                        follower.followPath(goTosample3From2, true);

                        outtakeSubsystem.goToTransfer();
                        //lift.goToGround();
                        intakeSubsystem.goDown();
                        intakeSubsystem.rotation.goToAutoPos();
                        intakeSubsystem.claw.open();

                        timer.reset();
                        NS = STATES.COLLECTING3;
                        CS = STATES.MOVING;
                    }
                }
                if(timer.milliseconds() > timeToCollect2 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.collect(true);
                    timer.reset();
                }
                break;

            case BASKET2:
                lift.goToHighBasket();
                if(firstTime) {
                    offf = false;
                    timer.reset();
                    firstTime = false;
                }
                if(timer.milliseconds() > time_to_lift) {
                    outtakeSubsystem.goToSampleScore();
                }

                if(timer.milliseconds() > time_to_drop) {
                    outtakeSubsystem.claw.open();
                    timer.reset();
                    TIME_TO_WAIT = timeToSample;
                    NS = STATES.SAMPLE3;
                    CS = STATES.WAITING;
                }
                break;

            case SAMPLE3:
                follower.setMaxPower(fastSpeed);
                follower.followPath(goTo3Sample, true);
                outtakeSubsystem.goToTransfer();
                //lift.goToGround();
                intakeSubsystem.goDown();
                intakeSubsystem.rotation.goToAutoPos();
                intakeSubsystem.claw.open();

                timer.reset();
                NS = STATES.COLLECTING3;
                CS = STATES.MOVING;
                break;

            case COLLECTING3:
                if(firstTime) {
                    timer.reset();
                    firstTime = false;
                    extendo.goToPos(extendoPoz3);
                }
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.COLECT_GOING_UP || intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) {
                    if(intakeSubsystem.hasElement() || offf == true) {
                        follower.setMaxPower(fastSpeed);
                        follower.followPath(goTo3Basket, true);
                        //robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN; //start transfer
                        NS = STATES.BASKET3;
                        firstTime = true;
                        CS = STATES.TRANSFERING;
                    } else {
                        timer.reset();
                        firstTime = true;
                        intakeSubsystem.goDown();
                        intakeSubsystem.claw.open();
                        offf = true;
                    }
                }
                if(timer.milliseconds() > timeToCollect3 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.collect(true);
                    timer.reset();
                }
                break;

            case BASKET3:
                lift.goToHighBasket();
                if(firstTime) {
                    timer.reset();
                    firstTime = false;
                }

                if(timer.milliseconds() > time_to_lift) {
                    outtakeSubsystem.goToSampleScore();
                }

                if(timer.milliseconds() > time_to_drop) {
                    outtakeSubsystem.claw.open();
                    timer.reset();
                    TIME_TO_WAIT = timeToSample;
                    NS = STATES.GO_SUBMERSIBLE;
                    CS = STATES.WAITING;
                }
                break;

            case GO_SUBMERSIBLE:
                follower.setMaxPower(fastSpeed);

                follower.followPath(goToSubmersible, true);
                outtakeSubsystem.goToTransfer();
                //lift.goToGround();
                extendo.goToPos(45);
                intakeSubsystem.goDown();
                intakeSubsystem.claw.open();

                timer.reset();
                NS = STATES.COLLECTING_SUBMERSIBLE;
                CS = STATES.MOVING;
                loopCount = 0;
                break;

            case COLLECTING_SUBMERSIBLE:
                extendo.moveFree(0.29);
                intakeSubsystem.goToCollectSubmersible();
                if(extendo.currentPos > 90) {
                    result = limelight.getLatestResult();
                    pythonOutput = result.getPythonOutput();
                    validContours = (int) pythonOutput[0];  // 1 dacă există contururi, 0 altfel
                    if(validContours == 1) loopCount ++; else loopCount = 0;
                    heading = pythonOutput[5];// Unghiul conturului
                }


                if(loopCount > 2) {
                    intakeSubsystem.rotation.rotateToAngle(90-(int) heading);
                    extendo.goToPos((int)((double)extendo.currentPos * 0.8));
                    CS = STATES.COLLECTING_SUBMERSIBLE2;
                    timer.reset();
                } else if(extendo.currentPos > 440 || timer.milliseconds() > timeTryingToCollect) {
                    extendo.goToPos(65);
                    intakeSubsystem.goDown();
                    CS = STATES.COLLECTING_SUBMERSIBLE2;
                }

                break;

            case COLLECTING_SUBMERSIBLE2:

                result = limelight.getLatestResult();
                pythonOutput = result.getPythonOutput();
                validContours = (int) pythonOutput[0];  // 1 dacă există contururi, 0 altfel
                if(validContours == 1) loopCount ++; else loopCount = 0;
                heading = pythonOutput[5];// Unghiul conturului

                if(loopCount > 2) {
                    intakeSubsystem.rotation.rotateToAngle(90-(int) heading);
                }

                if(extendo.isAtPosition() && timer.milliseconds() > timeGlis) {
                    intakeSubsystem.collect(true);
                    TIME_TO_WAIT = timeToCheckCollect;
                    CS = STATES.CHECK_COLLECT_SUBMERSIBLE;
                }
                break;


            case CHECK_COLLECT_SUBMERSIBLE:
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.COLECT_GOING_UP) {
                    SUBMERSIBLE_Y = SUBMERSIBLE2_Y;
                    SUBMERSIBLE2_Y += 2.2;
                    if(intakeSubsystem.hasElement()) {
                        //robotSystems.startTransfer(false);

                        goToBasketFromSub = new Path(
                                new BezierCurve(
                                        new Point(SUBMERSIBLE_X, SUBMERSIBLE_Y, Point.CARTESIAN),
                                        new Point(SAFE_X, SAFE_Y, Point.CARTESIAN),
                                        new Point(BASKET3_X, BASKET3_Y, Point.CARTESIAN)
                                )
                        );
                        goToBasketFromSub.setLinearHeadingInterpolation(Math.toRadians(SUBMERSIBLE_ANGLE), Math.toRadians(BASKET3_ANGLE));

                        goToSubmersible = new Path(
                                new BezierCurve(
                                        new Point(BASKET3_X, BASKET3_Y, Point.CARTESIAN),
                                        new Point(SAFE_X, SAFE_Y, Point.CARTESIAN),
                                        new Point(SUBMERSIBLE_X, SUBMERSIBLE_Y, Point.CARTESIAN)
                                )
                        );
                        goToSubmersible.setLinearHeadingInterpolation(Math.toRadians(BASKET3_ANGLE), Math.toRadians(SUBMERSIBLE_ANGLE));

                        follower.followPath(goToBasketFromSub, true);
                        CS = STATES.TRANSFERING;
                        NS = STATES.BASKET3;
                    } else {
                        goToSubmersible2 = new Path(
                                new BezierCurve(
                                        new Point(SUBMERSIBLE_X, SUBMERSIBLE_Y, Point.CARTESIAN),
                                        new Point(SUBMERSIBLE2_X, SUBMERSIBLE2_Y, Point.CARTESIAN)
                                )
                        );
                        goToSubmersible2.setConstantHeadingInterpolation(Math.toRadians(SUBMERSIBLE2_ANGLE));
                        intakeSubsystem.goDown();
                        intakeSubsystem.rotation.goToFlipped();
                        follower.followPath(goToSubmersible2, true);
                        CS = STATES.MOVING;
                        NS = STATES.COLLECTING_SUBMERSIBLE;
                    }
                }
                break;

        }

        telemetry.addData("Match Time", matchTimer.seconds());
        telemetry.addData("Current State", CS);
        telemetry.addData("Next State", follower.getPose());
        telemetry.addData("Transfer State", robotSystems.transferState);
        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("extendoTarget", extendo.target_position);
        telemetry.update();
        //follower.telemetryDebug(telemetryA);
    }


    public boolean isAtPos(Pose current, Pose target, double tolerance) { //i dont know if this works
        return Math.abs(current.getX() - target.getX()) < tolerance &&
                Math.abs(current.getY() - target.getY()) < tolerance;
    }
}
