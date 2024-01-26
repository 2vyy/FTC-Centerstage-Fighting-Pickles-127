package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "autoRewrite")
public class autoRewrite extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor motor;
    private DcMotor extendArm;
    private Servo arm;
    private Servo claw;
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    SamplePipeline.PropPosition snapshotAnalysis = SamplePipeline.PropPosition.CENTER;

    private final int TICKS_PER_REV = 28 * 20;
    private final double CIRCUMFERENCE = 10.99;
    private final double TICKS_PER_INCH = TICKS_PER_REV/CIRCUMFERENCE;

    @Override
    public void runOpMode() {
        //camInit(1); // ** 1 for red, 2 for blue **

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        extendArm = hardwareMap.get(DcMotor.class, "hexMotor");
        arm = hardwareMap.get(Servo.class, "arm1");
        motor = hardwareMap.get(DcMotor.class, "hexMotor");
        claw = hardwareMap.get(Servo.class, "claw");


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //replaces waitForStart
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles
            sleep(50);
        }

        //snapshotAnalysis = pipeline.getAnalysis();
        //webcam.stopStreaming();
        //webcam.closeCameraDevice();
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();



    }

    //1 for red, 2 for blue
    public void camInit(int teamColor) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("webcam", "webcam", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        pipeline = new SamplePipeline();
        pipeline.setTeamColor(teamColor); // ** 1 for red, 2 for blue **
        webcam.setPipeline(pipeline);
        //webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // TODO: make sure resolution is highest that is supported by webcam
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//                //0 for blue, 1 for red (maybe)
//                //webcam.setTeamColor(0);
//                snapshotAnalysis = pipeline.getAnalysis();
//                webcam.stopStreaming();
//                webcam.closeCameraDevice();
//                telemetry.addLine("Analysis: "+snapshotAnalysis);
            }
            @Override
            public void onError(int errorCode) {
                //called if cam cannot be opened
                snapshotAnalysis = SamplePipeline.PropPosition.CENTER;
                telemetry.addLine("Analysis Failed\nError: "+errorCode+"\nDefaulting to Center");
            }
        });
    }

    public void drive(double inches) { // a little off, might need increase
        if(inches<1) {
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
        } else {
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition((int) (inches*TICKS_PER_INCH));
        leftDrive.setTargetPosition((int) (inches*TICKS_PER_INCH));

        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);

        while(opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {}

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turnLeft() {
        leftDrive.setPower(0.4);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setPower(0.4);
        //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while(runtime.time()<0.12*7.5 && opModeIsActive()) {}
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

    }
    public void turnRight() {
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setPower(0.4);

        rightDrive.setPower(0.4);

        runtime.reset();
        while(runtime.time()<0.12*7.5  && opModeIsActive()) {}
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void openClaw() {
        claw.setPosition(.5);
    }
    public void closeClaw() {
        claw.setPosition(0);
    }

    public void dropOff(double inches) {
        drive(inches);
        drive(inches);
    }

    public void waitASec() {
        runtime.reset();
        while(runtime.time()<1.5 && opModeIsActive()) {}
    }
}
