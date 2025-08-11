package pedroPathing;

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



import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "auton", group = "Examples")
public class auton extends OpMode {

    private PIDController controller;
    NormalizedColorSensor colorSensor;
    float[] hsvValues = new float[3];

    public static double p = 0.0009, i = 0, d = 0.0001;
    public static double f = 0.0005;
    public static double open_pos = 0.35, close_pos = 0.125;
    public static double normal_pos = 1, upside_pos = 0.32;
    public static int target = 0;
    public static double pick_x = 12;

    private final double ticks_in_degree = 2786.2 / 360.0;
    public static double push_up_pos = 24;
    public static int clip_pos= -1010;
    private DcMotor arm;
    private Servo clawservo;
    private Servo twist;

    int iftrue = 0;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(15, 62, Math.toRadians(180));
    private final Pose subpose = new Pose(24, 62, Math.toRadians(180));
    private final Pose finalPush = new Pose(28.5,62, Math.toRadians(180));
    private final Pose finalback = new Pose(21.5, 62, Math.toRadians(180));
    private final Pose hello = new Pose(8,68, Math.toRadians(180));
    private final Pose subcontrol = new Pose(32, 32, Math.toRadians(180));
    private final Pose pos1 = new Pose(26, 27, Math.toRadians(180));
    private final Pose push_blocks = new Pose(56, 36, Math.toRadians(180));
    private final Pose pos1_control = new Pose(56, 25.5, Math.toRadians(180));
    private final Pose pos2_final = new Pose(26, 18, Math.toRadians(180));
    private final Pose pos2 = new Pose(56,18, Math.toRadians(180));
    private final Pose pos2_control = new Pose(56, 25, Math.toRadians(180));
    private final Pose pose3 = new Pose(56,18, Math.toRadians(180));
    private final Pose pose3_control = new Pose(56,10, Math.toRadians(180));
    private final Pose pose3_final = new Pose(pick_x,12.25, Math.toRadians(180));
    private final Pose dep2_final= new Pose(24,68, Math.toRadians(180));
    private Pose dep2_push = new Pose(29,68,Math.toRadians(180));
    private Pose pick_pos = new Pose(pick_x,32,Math.toRadians(180));
    private Pose pick3_control = new Pose(8,68,Math.toRadians(180));
    private Pose dep3 = new Pose(24,70,Math.toRadians(180));
    private Pose dep3_push = new Pose(29,70,Math.toRadians(180));
    private Pose dep3_control= new Pose(8,70,Math.toRadians(180));
    private Pose pick4_control = new Pose(8,70,Math.toRadians(180));
    private Pose dep4 = new Pose(24,72, Math.toRadians(180));
    private Pose dep4_push = new Pose(29,72,Math.toRadians(180));
    private Pose dep4_control = new Pose(8,72, Math.toRadians(180));
    private Pose pick5_control = new Pose(8,72, Math.toRadians(180));
    private Pose dep5_control = new Pose(8,74, Math.toRadians(180));
    private Pose dep5 =new Pose(24,74,Math.toRadians(180));
    private Pose dep5_push = new Pose(29,74,Math.toRadians(180));
    private Path move;
    private Path Finish_him;
    private Path move_back;
    private Path first_block1;
    private Path first_block;
    private PathChain block;
    private Path seconds_block;
    private Path push_2;
    private Path push_3;
    private Path push_third;
    private Path dep2;
    private Path pickup_3;
    private Path score3;
    private Path pickup_4;
    private Path score4;
    private Path pickup5;
    private Path score5;

    private Path finish3;
    private Path finish4;
    private Path finish5;

    private Path finish2;

    public void buildPaths() {
        move = new Path(new BezierLine(new Point(startPose), new Point(subpose)));
        move.setLinearHeadingInterpolation(startPose.getHeading(), subpose.getHeading());
        Finish_him = new Path(new BezierLine(new Point(subpose), new Point(finalPush)));
        Finish_him.setLinearHeadingInterpolation(subpose.getHeading(), finalPush.getHeading());

        dep2 = new Path(new BezierCurve( new Point(pose3_final),new Point(hello), new Point(dep2_final)));
        dep2.setLinearHeadingInterpolation(pose3_final.getHeading(), dep2_final.getHeading());
        finish2 = new Path(new BezierLine(new Point(dep2_final), new Point(dep2_push)));
        finish2.setLinearHeadingInterpolation(dep2_final.getHeading(), dep2_push.getHeading());

        pickup_3 = new Path(new BezierCurve( new Point(dep2_final),new Point(pick3_control), new Point(pick_pos)));
        pickup_3.setLinearHeadingInterpolation(dep2_final.getHeading(), pick_pos.getHeading());

        score3 = new Path(new BezierCurve( new Point(pick_pos),new Point(dep3_control), new Point(dep3)));
        score3.setLinearHeadingInterpolation(pick_pos.getHeading(), dep3.getHeading());
        finish3 = new Path(new BezierLine(new Point(dep3), new Point(dep3_push)));
        finish3.setLinearHeadingInterpolation(dep3.getHeading(), dep3_push.getHeading());

        finish4 = new Path(new BezierLine(new Point(dep4), new Point(dep4_push)));
        finish4.setLinearHeadingInterpolation(dep4.getHeading(), dep4_push.getHeading());

        finish5 = new Path(new BezierLine(new Point(dep5), new Point(dep5_push)));
        finish5.setLinearHeadingInterpolation(dep5.getHeading(), dep5_push.getHeading());

        pickup_4 = new Path(new BezierCurve( new Point(dep3),new Point(pick4_control), new Point(pick_pos)));
        pickup_4.setLinearHeadingInterpolation(dep3.getHeading(), pick_pos.getHeading());

        score4 = new Path(new BezierCurve( new Point(pick_pos),new Point(dep4_control), new Point(dep4)));
        score4.setLinearHeadingInterpolation(pick_pos.getHeading(), dep4.getHeading());

        pickup5 = new Path(new BezierCurve( new Point(dep4),new Point(pick5_control), new Point(pick_pos)));
        pickup5.setLinearHeadingInterpolation(dep4.getHeading(), pick_pos.getHeading());

        score5 = new Path(new BezierCurve( new Point(pick_pos),new Point(dep5_control), new Point(dep5)));
        score5.setLinearHeadingInterpolation(pick_pos.getHeading(), dep5.getHeading());

        block = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(finalPush), new Point(finalback)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(new Point(finalback), new Point(subcontrol), new Point(push_blocks)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(
                        new BezierCurve(new Point(push_blocks), new Point(pos1_control), new Point(pos1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(new Point(pos1), new Point(pos2_control)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(new Point(pos2_control), new Point(pos2),new Point(pos2_final)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine( new Point(pos2_final), new Point(pose3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(new Point(pose3), new Point(pose3_control), new Point(pose3_final)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(move);
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    target = -1180;
                    twist.setPosition(upside_pos);
                    pathTimer.resetTimer();
                    setPathState(1);
                }
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                    follower.followPath(Finish_him);
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    clawservo.setPosition(open_pos);
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    twist.setPosition(normal_pos);
                    follower.followPath(block);
                    target=-180;
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1) {

                    target=-180;
                    setPathState(4);

                }



                break;

            case 4:
                // Use color + distance sensor check here
                if (isTargetColorClose(15, 60, 2.3)) {
                    follower.breakFollowing();
                    clawservo.setPosition(close_pos);
                    target = arm.getCurrentPosition(); // Hold arm at current position
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()& pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = clip_pos;
                    follower.followPath(dep2);
                    twist.setPosition(upside_pos);
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                    follower.followPath(finish2);
                    target = -1050;
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    clawservo.setPosition(open_pos);
                    twist.setPosition(normal_pos);
                    follower.followPath(pickup_3);
                    target = -1230;
                    setPathState(8);
                }
                break;

            case 8:
                if (follower.getPose().getX() < 24) {
                    target = -180;
                    setPathState(10);
                }
                break;

            case 10:
                if (isTargetColorClose(15, 60, 3)) {
                    clawservo.setPosition(close_pos);
                    setPathState(-1);
                }
                break;

            case 11:
                if (!follower.isBusy()& pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = clip_pos;
                    follower.followPath(score3);
                    twist.setPosition(upside_pos);

                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                    follower.followPath(finish3);
                    target = -1270;
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    clawservo.setPosition(open_pos);
                    twist.setPosition(normal_pos);
                    target = -1300;
                    follower.followPath(pickup_4);
                    setPathState(14);
                }
                break;

            case 14:
                if (follower.getPose().getX() < 25) {
                    target = -135;
                    setPathState(15);
                }
                break;

            case 15:
                if (follower.getPose().getX() < 12.3) {
                    clawservo.setPosition(close_pos);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    clawservo.setPosition(close_pos);
                    target = clip_pos;
                    follower.followPath(score4);
                    twist.setPosition(upside_pos);
                    setPathState(17);
                }
                break;

            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                    follower.followPath(finish4);
                    target = -1270;
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    clawservo.setPosition(open_pos);
                    twist.setPosition(normal_pos);
                    target = -1300;
                    follower.followPath(pickup5);
                    setPathState(19);
                }
                break;

            case 19:
                if (follower.getPose().getX() < 12.3) {
                    clawservo.setPosition(close_pos);
                    setPathState(20);
                }
                break;

            case 20:
                if (follower.getPose().getX() < 24) {
                    target = -135;
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    clawservo.setPosition(close_pos);
                    target = clip_pos;
                    follower.followPath(score5);
                    twist.setPosition(upside_pos);
                    setPathState(22);
                }
                break;

            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                    follower.followPath(finish5);
                    target = -1270;
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;

            case 25:
                if (!follower.isBusy()) {
                    setPathState(21);
                }
                break;

            case 26:
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;

            case 27:
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

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        twist = hardwareMap.get(Servo.class, "twist");
        twist.setPosition(normal_pos);
        clawservo = hardwareMap.get(Servo.class, "claw_servo");
        clawservo.setPosition(close_pos);

        controller = new PIDController(p, i, d);
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
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}

    // Helper method for color and distance check
    private boolean isTargetColorClose(float minHue, float maxHue, double maxDistanceCm) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];

        double distance = 999;
        if (colorSensor instanceof com.qualcomm.robotcore.hardware.DistanceSensor) {
            distance = ((com.qualcomm.robotcore.hardware.DistanceSensor) colorSensor)
                    .getDistance(DistanceUnit.CM);


        }

        return (hue >= minHue && hue <= maxHue) && (distance <= maxDistanceCm);
    }
}
