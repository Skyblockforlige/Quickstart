package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "bucket_auton", group = "Examples")
public class bucket_auton extends OpMode {

    private pedroPathing.PIDController controller;
    public static double p = 0.0009, i = 0, d = 0.0001;
    public static double f = 0.0005;
    public static double open_pos = 0.4, close_pos = 0.125;
    public static double normal_pos = 1, upside_pos = 0.32, middle = 0.66;
    public static int target = 0;

    private DcMotor arm;
    private Servo clawservo;
    private Servo twist;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(10, 110, Math.toRadians(270));
    private final Pose dep_1 = new Pose(10, 118, Math.toRadians(270));
    private final Pose pick2_pos = new Pose(31, 112, Math.toRadians(180));
    private final Pose Dep_2 = new Pose(20,124, Math.toRadians(315));
    private final Pose pick3_pos = new Pose(31,126, Math.toRadians(180));
    private final Pose pick4_pos = new Pose(48, 124, Math.toRadians(270));
    private final Pose pick4_back = new Pose(48,120, Math.toRadians(270));

    private PathChain score1;
    private Path pick2;
    private Path score2;
    private Path pick3;
    private Path score3;
    private Path pick4;
    private Path score4;
    private Path move4;

    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(dep_1)))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        pick2 = new Path(new BezierLine(new Point(dep_1), new Point(pick2_pos)));
        pick2.setLinearHeadingInterpolation(dep_1.getHeading(), pick2_pos.getHeading());

        score2 = new Path(new BezierLine(new Point(pick2_pos), new Point(Dep_2)));
        score2.setLinearHeadingInterpolation(pick2_pos.getHeading(), Dep_2.getHeading());

        pick3 = new Path(new BezierLine(new Point(Dep_2), new Point(pick3_pos)));
        pick3.setLinearHeadingInterpolation(Dep_2.getHeading(), pick3_pos.getHeading());

        score3 = new Path(new BezierLine(new Point(pick3_pos), new Point(Dep_2)));
        score3.setLinearHeadingInterpolation(pick3_pos.getHeading(), Dep_2.getHeading());

        pick4 = new Path(new BezierLine(new Point(Dep_2), new Point(pick4_pos)));
        pick4.setLinearHeadingInterpolation(Dep_2.getHeading(), pick4_pos.getHeading());

        move4 = new Path(new BezierLine(new Point(pick4_back), new Point(Dep_2)));
        move4.setLinearHeadingInterpolation(pick4_back.getHeading(), Dep_2.getHeading());

        score4 = new Path(new BezierLine(new Point(pick4_pos), new Point(Dep_2)));
        score4.setLinearHeadingInterpolation(pick4_pos.getHeading(), Dep_2.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(score1);
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    target = -870;
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy())
                    clawservo.setPosition(open_pos);

                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pick2);
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = -1550;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8)
                    setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (Math.abs(arm.getCurrentPosition() - (-1550)) < 25) {
                        clawservo.setPosition(close_pos);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        target = -870;

                        if (pathTimer.getElapsedTimeSeconds() > 7) {
                            follower.followPath(score2);
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (Math.abs(arm.getCurrentPosition() - (-870)) < 25) {
                        clawservo.setPosition(open_pos);
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            follower.followPath(pick3);
                            setPathState(5);
                        }
                    }
                }
                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds()>5) {
                    target = -1550;
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    if (Math.abs(arm.getCurrentPosition() - (-1550)) < 25) {
                        clawservo.setPosition(close_pos);
                        if (pathTimer.getElapsedTimeSeconds() > 5) {
                            target = -870;
                        }
                            if (pathTimer.getElapsedTimeSeconds() > 6) {
                                follower.followPath(score3);
                                setPathState(7);

                        }
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    if (Math.abs(arm.getCurrentPosition() - (-870)) < 25) {
                        clawservo.setPosition(open_pos);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(pick4);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds()>1.5) {
                    target = -1550;
                    twist.setPosition(middle);
                    if (!follower.isBusy()) {
                        if (Math.abs(arm.getCurrentPosition() - (-1550)) < 25) {
                            clawservo.setPosition(close_pos);
                            if (pathTimer.getElapsedTimeSeconds() > 1) {
                                follower.followPath(move4);

                                setPathState(9);
                            }
                        }
                    }
                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    target = -870;
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(score4);
                    if (!follower.isBusy()&Math.abs(arm.getCurrentPosition() - (-870)) < 25) {
                        clawservo.setPosition(open_pos);
                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                            setPathState(12);
                        }
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
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
    public void init_loop() {}

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
}
