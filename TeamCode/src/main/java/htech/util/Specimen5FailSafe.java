package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.config.PositionsLift;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "[AUTO] 5+0 Fail Safe", group = "HTECH")
public class Specimen5FailSafe extends LinearOpMode {
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    Follower follower;

    Path preload;
    PathChain collectSamples, failSafe1, failSafe;
    Path score1, score2, score3, score4;
    Path wall;
    Path park;
    Path scoreSpecimen, scoreSpecimen2;



    public enum STATES{
        IDLE,
        SPECIMEN, SCORING_SPECIMEN,
        COLLECTING_SAMPLES, COLLECTING_SPECIMEN,
        SCORE,
        WALL,
        PARK,
        PARKED,
        MOVING, WAITING, TRANSFER,
        CHECK_SPECIMEN, FAIL_SAFE;
        //FAIL_SAFE
    }
    public enum SCORING_STATES{
        IDLE,
        SCORE1,
        SCORE2,
        SCORE3,
        SCORE4
    }
    STATES CS = STATES.IDLE, NS = STATES.IDLE;
    SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;

    public static double startX = 0, startY = 0, startH = 0;
    public static double preloadX = -26, preloadY = -2.7, preloadH = startH;

    public static double safe1Sample1X = -5, safe1Sample1Y = 32;
    public static double safe2Sample1X = -30, safe2Sample1Y = 15;
    public static double safe3Sample1X = -60, safe3Sample1Y = 37;
    public static double sample1X = -48, sample1Y = 37, sample1H = 0;
    public static double human1X = -23, human1Y = 37, human1H = 0;

    public static double safeSample2X = -48, safeSample2Y = 30;
    public static double sample2X = -48, sample2Y = 45, sample2H = 0;
    public static double human2X = -25.5, human2Y = 45, human2H = 0;

    public static double safeSample3X = -35, safeSample3Y = 37;
    public static double sample3X = -48, sample3Y = 52.7, sample3H = 0;
    public static double specimen1X = -8.8, specimen1Y = 52.7, specimen1H = 0;

    public static double score1X = -26, score1Y = 1, scoreH = 0;
    public static double score2X = -26, score2Y = -1;
    public static double score3X = -26, score3Y = -6;
    public static double score4X = -26, score4Y = -8;
    public static double safeScoreX = -14, safeScoreY = 0;

    public static double specimenX = -11.5, specimenY = 30, specimenH = 0;
    public static double safe1SpecimenX = -20, safe1SpecimenY = 5;
    public static double safe2SpecimenX = -20, safe2SpecimenY = 30;

    public static double parkX = -10, parkY = 30, parkH = 80;


    public static double time_to_start = 0;
    public static double timeToTransfer = 700;
    public static double timeToTransfer1 = 700;
    public static double timeToCollect = 100;
    public static double timeToCheck = 0;
    public static double timeToScoreSpecimen = 600;
    public static double timeToScoreSpecimenVertical = 380;
    public double timeToWait = 0;

    public static double maxSpeed = 1;
    public static double collectSpeed = 0.6;
    public static double mediumSpeed = 1;
    public static double slowSpeed = 0.7;

    public static double timeout = 255;

    public Pose curr;

    public boolean firstSpecimen = true;

    public static double magicScore = 1;
    public static double magicScore2 = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        curr = new Pose(0,0,0);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, startH));

        intakeSubsystem.initAuto();
        outtakeSubsystem.init();
        outtakeSubsystem.claw.close();
        extendo.pidEnabled = true;

        preload = new Path(
                new BezierLine(
                        new Point(startX, startY, Point.CARTESIAN),
                        new Point(preloadX, preloadY, Point.CARTESIAN)
                )
        );
        preload.setConstantHeadingInterpolation(preloadH);

        collectSamples = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(preloadX, preloadY, Point.CARTESIAN),
                                new Point(safe1Sample1X, safe1Sample1Y, Point.CARTESIAN),
                                new Point(safe2Sample1X, safe2Sample1Y, Point.CARTESIAN),
                                new Point(safe3Sample1X, safe3Sample1Y, Point.CARTESIAN),
                                new Point(sample1X, sample1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample1X, sample1Y, Point.CARTESIAN),
                                new Point(human1X, human1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human1X, human1Y, Point.CARTESIAN),
                                new Point(safeSample2X, safeSample2Y, Point.CARTESIAN),
                                new Point(sample2X, sample2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample2X, sample2Y, Point.CARTESIAN),
                                new Point(human2X, human2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human2X, human2Y, Point.CARTESIAN),
                                new Point(safeSample3X, safeSample3Y, Point.CARTESIAN),
                                new Point(sample3X, sample3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample3H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(sample3X, sample3Y, Point.CARTESIAN),
                                new Point(specimen1X, specimen1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(specimen1H))
                .setPathEndTimeoutConstraint(0)
                .build();

        score1 = new Path(
                new BezierCurve(
                        new Point(specimen1X, specimen1Y, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score1X, score1Y, Point.CARTESIAN)
                )
        );
        score1.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score2 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score2X, score2Y, Point.CARTESIAN)
                )
        );
        score2.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score3 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score3X, score3Y, Point.CARTESIAN)
                )
        );
        score3.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score4 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score4X, score4Y, Point.CARTESIAN)
                )
        );
        score4.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        wall = new Path(
                new BezierCurve(
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(safe1SpecimenX, safe1SpecimenY, Point.CARTESIAN),
                        new Point(safe2SpecimenX, safe2SpecimenY, Point.CARTESIAN),
                        new Point(specimenX, specimenY, Point.CARTESIAN)
                )
        );
        wall.setConstantHeadingInterpolation(Math.toRadians(specimenH));
        wall.setPathEndTimeoutConstraint(timeout);

        failSafe1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimen1X, specimen1Y, Point.CARTESIAN),
                                new Point(specimen1X - 5, specimen1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(200)
                .addPath(
                        new BezierLine(
                                new Point(specimen1X - 5, specimen1Y, Point.CARTESIAN),
                                new Point(specimen1X + 1, specimen1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(0)
                .build();

        failSafe = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenX, specimenY, Point.CARTESIAN),
                                new Point(specimenX - 5, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(300)
                .addPath(
                        new BezierLine(
                                new Point(specimenX - 5, specimenY, Point.CARTESIAN),
                                new Point(specimenX + 1, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(0)
                .build();

        park = new Path(
                new BezierLine(
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(parkX, parkY, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(parkH));

        scoreSpecimen = new Path(
                new BezierLine(
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(preloadX + magicScore, preloadY, Point.CARTESIAN)
                )
        );
        scoreSpecimen.setConstantHeadingInterpolation(preloadH);

        scoreSpecimen2 = new Path(
                new BezierLine(
                        new Point(score3X, score3Y, Point.CARTESIAN),
                        new Point(score3X + magicScore2, score3Y, Point.CARTESIAN)
                )
        );
        scoreSpecimen2.setConstantHeadingInterpolation(preloadH);


        follower.setMaxPower(maxSpeed);
        lift.goToSpecimenVertical();

        waitForStart();


        matchTimer.reset();
        while(opModeIsActive() && matchTimer.seconds() < 30.5){

            switch (CS){

                case IDLE:
                    if(timer.milliseconds() > time_to_start) {
                        follower.followPath(preload);
                    }
                    CS = STATES.SPECIMEN;
                    outtakeSubsystem.goToSpecimenVertical();
                    break;

                case SPECIMEN:
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE || robotSystems.transferState == RobotSystems.TransferStates.CATCHING) {

                        lift.goToSpecimenVertical();

                        //outtakeSubsystem.goToSpecimenScore();
                        CS = STATES.MOVING;
                        NS = STATES.SCORING_SPECIMEN;
                    }
                    break;

                case SCORING_SPECIMEN:
                    follower.setMaxPower(maxSpeed);
                    timeToWait = timeToScoreSpecimenVertical;
                    CS = STATES.WAITING;
                    timer.reset();
                    robotSystems.placeVertical();
                    switch (SCORING_CS){
                        case IDLE:
                            NS = STATES.COLLECTING_SAMPLES;
                            break;
                        case SCORE1:
                            NS = STATES.WALL;
                            break;
                        case SCORE2:
                            NS = STATES.WALL;
                            break;
                        case SCORE3:
                            NS = STATES.WALL;
                            break;
                        case SCORE4:
                            NS = STATES.PARK;
                            break;
                    }
                    break;


                case MOVING:
                    if(!follower.isBusy()){
                        CS = NS;
                    }
                    break;

                case WAITING:
                    if(timer.milliseconds() > timeToWait){
                        CS = NS;
                        timer.reset();
                    }
                    break;

                case COLLECTING_SAMPLES:
                    timer.reset();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(collectSamples);
                    lift.goToGround();
                    outtakeSubsystem.claw.open();
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SPECIMEN;
                    intakeSubsystem.goToWall();
                    intakeSubsystem.claw.open();
                    SCORING_CS = SCORING_STATES.SCORE1;
                    break;

                case COLLECTING_SPECIMEN:
                    intakeSubsystem.claw.close();
                    timer.reset();
                    timeToWait = timeToCollect;
                    CS = STATES.WAITING;
                    NS = STATES.CHECK_SPECIMEN;
                    break;

                case CHECK_SPECIMEN:
                    if(timer.milliseconds() > timeToCheck) {
                        if(intakeSubsystem.hasElement()){
                            CS = STATES.TRANSFER;
                            robotSystems.startTransfer(false);
                        }
                        else{
                            intakeSubsystem.claw.open();
                            timer.reset();
                            timeToWait = 100;
                            CS = STATES.WAITING;
                            NS = STATES.FAIL_SAFE;
                        }
                    }

                    break;

                case FAIL_SAFE:
                    if(SCORING_CS == SCORING_STATES.SCORE1){
                        follower.setMaxPower(maxSpeed);
                        follower.followPath(failSafe1);
                        CS = STATES.MOVING;
                        NS = STATES.COLLECTING_SPECIMEN;
                    }
                    else{
                        follower.setMaxPower(mediumSpeed);
                        follower.followPath(failSafe);
                        CS = STATES.MOVING;
                        NS = STATES.COLLECTING_SPECIMEN;
                    }
                    break;


                case TRANSFER:
                    if(robotSystems.transferState == RobotSystems.TransferStates.WAITING_TO_CATCH) {
                        CS = STATES.SCORE;
                    } else if(!intakeSubsystem.hasElement() && !outtakeSubsystem.hasElement()) {
                        intakeSubsystem.claw.open();
                        timer.reset();
                        timeToWait = 100;
                        CS = STATES.WAITING;
                        NS = STATES.FAIL_SAFE;
                    }
                    break;

                case SCORE:
                    follower.setMaxPower(maxSpeed);
                    switch (SCORING_CS){
                        case SCORE1:
                            follower.followPath(score1);
                            break;
                        case SCORE2:
                            follower.followPath(score2);
                            break;
                        case SCORE3:
                            follower.followPath(score3);
                            break;
                        case SCORE4:
                            follower.followPath(score4);
                            break;
                    }
                    CS = STATES.SPECIMEN;
                    break;

                case WALL:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(wall, false);
                    lift.goToGround();
                    outtakeSubsystem.claw.open();
                    switch (SCORING_CS) {
                        case SCORE1:
                            SCORING_CS = SCORING_STATES.SCORE2;
                            break;
                        case SCORE2:
                            SCORING_CS = SCORING_STATES.SCORE3;
                            break;
                        case SCORE3:
                            SCORING_CS = SCORING_STATES.SCORE4;
                            break;
                    }
                    intakeSubsystem.goToWall();
                    intakeSubsystem.claw.open();
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SPECIMEN;
                    break;

                case PARK:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(park, false);
                    lift.goToGround();
                    outtakeSubsystem.claw.open();
                    extendo.goToGround();
                    CS = STATES.PARKED;
                    break;

                case PARKED:
                    break;
            }

            follower.update();
            robotSystems.update();
            telemetry.addData("STATE", CS);
            telemetry.addData("TIMER", timer.milliseconds());
            telemetry.update();
        }
    }
}
