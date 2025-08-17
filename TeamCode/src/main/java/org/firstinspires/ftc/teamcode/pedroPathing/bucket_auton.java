package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@Autonomous(name = "bucket_auton", group = "Examples")
public class bucket_auton extends OpMode {

    private pedroPathing.PIDController controller;
    public static double p = 0.0009, i = 0, d = 0.0001;
    public static double f = 0.0005;
    public static double open_pos = 0.4, close_pos = 0.125;
    public static double normal_pos = 1, upside_pos = 0.32;
    public static int target = 0;



    private DcMotor arm;
    private Servo clawservo;
    private Servo twist;

    int iftrue = 0;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(10, 110, Math.toRadians(270));
    private final Pose dep_1 = new Pose(10, 118, Math.toRadians(270));
    private final Pose pick2_pos = new Pose(29, 112, Math.toRadians(180));


    private PathChain score1;
    private Path pick2;

    public void buildPaths() {

        //score1 = new Path(new BezierLine( new Point(startPose),new Point(dep_1)));
        //score1.setLinearHeadingInterpolation(startPose.getHeading(), dep_1.getHeading());

        /*
        score5 = new Path(new BezierCurve( new Point(pick_pos),new Point(dep5_control), new Point(dep5)));
        score5.setLinearHeadingInterpolation(pick_pos.getHeading(), dep5.getHeading());

        score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(startPose), new Point(dep_1)))
                .setConstantHeadingInterpolation(Math.toRadians(270))


                .build();
        pick2 = new Path(new BezierLine( new Point(dep_1),new Point(pick2_pos)));
        pick2.setLinearHeadingInterpolation(dep_1.getHeading(), pick2_pos.getHeading());


        public class GeneratedPaths {

            public static PathBuilder builder = new PathBuilder();

            public static PathChain line1 = builder
                    .addPath(
                            new BezierCurve(
                                    new Point(10.000, 110.000, Point.CARTESIAN),
                                    new Point(9.109, 122.100, Point.CARTESIAN),
                                    new Point(33.000, 120.000, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(null), Math.toRadians(0))
                    .build();

            public static PathChain line2 = builder
                    .addPath(
                            new BezierLine(
                                    new Point(33.000, 120.000, Point.CARTESIAN),
                                    new Point(14.923, 128.495, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                    .setReversed(true)
                    .build();

            public static PathChain line3 = builder
                    .addPath(
                            new BezierLine(
                                    new Point(14.923, 128.495, Point.CARTESIAN),
                                    new Point(34.366, 131.790, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                    .build();

            public static PathChain line4 = builder
                    .addPath(
                            new BezierLine(
                                    new Point(34.366, 131.790, Point.CARTESIAN),
                                    new Point(14.923, 128.495, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                    .build();

            public static PathChain line5 = builder
                    .addPath(
                            new BezierLine(
                                    new Point(14.923, 128.495, Point.CARTESIAN),
                                    new Point(32.172, 132.565, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45))
                    .build();

            public static PathChain line6 = builder
                    .addPath(
                            new BezierLine(
                                    new Point(32.172, 132.565, Point.CARTESIAN),
                                    new Point(14.923, 128.495, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
                    .build();

            public static PathChain line7 = builder
                    .addPath(
                            new BezierCurve(
                                    new Point(14.923, 128.495, Point.CARTESIAN),
                                    new Point(63.182, 131.790, Point.CARTESIAN),
                                    new Point(63.182, 94.579, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                    .setReversed(true)
                    .build();
        }

         */

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(score1);
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    target=-900;
                    setPathState(1);
                }
                break;

            case 1:
                if(follower.getPose().getX() > 116)
                    clawservo.setPosition(open_pos);
                if (!follower.isBusy()) {
                    follower.followPath(pick2);

                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    target=-1500;

                }
                if(pathTimer.getElapsedTimeSeconds()>0.8)
                    setPathState(3);

                break;

            case 3:
                if (!follower.isBusy()) {
                    clawservo.setPosition(close_pos);

                    setPathState(-1);
                }
                break;


        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        twist = hardwareMap.get(Servo.class, "twist");
        twist.setPosition(normal_pos);
        clawservo = hardwareMap.get(Servo.class, "claw_servo");
        clawservo.setPosition(close_pos);

        controller = new pedroPathing.PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Waiting for start...");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        int armPos = arm.getCurrentPosition();
        double power = controller.update(target, armPos) + f;
        power = Math.max(-1, Math.min(1, power));
        arm.setPower(power);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Arm Pos", armPos);
        telemetry.addData("Arm Target", target);
        telemetry.addData("Arm Power", power);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() {}

    // Helper method for color and distance check
}
