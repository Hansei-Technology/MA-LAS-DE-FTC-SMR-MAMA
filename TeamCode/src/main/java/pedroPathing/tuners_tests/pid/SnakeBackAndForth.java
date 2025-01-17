package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import htech.subsystem.ExtendoSystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Snake Back And Forth", group = "PIDF Tuning")
public class SnakeBackAndForth extends LinearOpMode {

    ExtendoSystem extendo;

    private Follower follower;

    private Path forwards;
    private Path backwards;

    private boolean forward = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);

        forwards = new Path(
                new BezierCurve(
                        new Point(0, 0, Point.CARTESIAN),
                        new Point(10, 10, Point.CARTESIAN),
                        new Point(20, -10, Point.CARTESIAN),
                        new Point(30, 10, Point.CARTESIAN),
                        new Point(40, 0, Point.CARTESIAN)
                )
        );
        forwards.setConstantHeadingInterpolation(0);

        backwards = new Path(
                new BezierCurve(
                        new Point(40, 0, Point.CARTESIAN),
                        new Point(30, 10, Point.CARTESIAN),
                        new Point(20, -10, Point.CARTESIAN),
                        new Point(10, 10, Point.CARTESIAN),
                        new Point(0, 0, Point.CARTESIAN)
                )
        );
        backwards.setConstantHeadingInterpolation(0);

        extendo.goToGround();

        waitForStart();

        while(opModeIsActive()){
            follower.update();
            if (!follower.isBusy()) {
                if (forward) {
                    forward = false;
                    follower.followPath(backwards);
                } else {
                    forward = true;
                    follower.followPath(forwards);
                }
            }

        }
    }
}
