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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;

public class autoaim extends OpMode {

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
    private final Pose startPose = new Pose(10, 110, Math.toRadians(0));
    private  Pose dep_1 = new Pose(10, 110.5, Math.toRadians(0));
    public Limelight3A limelight;
    private PathChain align;
    List<LLResultTypes.DetectorResult> detectorResults;




    public void buildPaths() {

        //score1 = new Path(new BezierLine( new Point(startPose),new Point(dep_1)));
        //score1.setLinearHeadingInterpolation(startPose.getHeading(), dep_1.getHeading());

        /*
        score5 = new Path(new BezierCurve( new Point(pick_pos),new Point(dep5_control), new Point(dep5)));
        score5.setLinearHeadingInterpolation(pick_pos.getHeading(), dep5.getHeading());
        */
        align = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(startPose), new Point(dep_1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))


                .build();


    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.update();
        controller = new pedroPathing.PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();


    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(align);
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    target=-900;
                }
                setPathState(0);
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            //Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());
                telemetry.addData("ta", result.getTa());


                detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }



            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
        if(result.getTx() < 4.6){
            dep_1.setY(dep_1.getY()+0.5);
            buildPaths();
        } else if (result.getTx() > 4.9) {
            dep_1.setY(dep_1.getY()-0.5);
        } else {
            stop();
        }
        telemetry.update();
        follower.update();

        autonomousPathUpdate();



        telemetry.addData("Path State", pathState);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

}
