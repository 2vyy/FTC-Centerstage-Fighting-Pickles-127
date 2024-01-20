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
        camInit(2);

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        extendArm = hardwareMap.get(DcMotor.class, "hexMotor");
        arm = hardwareMap.get(Servo.class, "arm1");
        claw = hardwareMap.get(Servo.class, "claw");


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //replaces waitForStart
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

//        drive(24,24);
//        switch (snapshotAnalysis) {
//            case LEFT: {
//                turnLeft();
//                dropOff(6);
//                turnRight();
//                drive(24,24);
//                turnRight();
//                drive(3*24,3*24);
//                turnRight();
//            }
//            case RIGHT: {
//                turnRight();
//                dropOff(6);
//                turnLeft();
//                drive(24,24);
//                turnRight();
//                drive(3*24,3*24);
//                turnRight();
//            }
//            default: {
//                drive(24,24);
//                turnRight();
//                turnRight();
//                dropOff(6);
//                turnLeft();
//                drive(3*24,3*24);
//                turnRight();
//            }
//        }
//
//        drive(12,12);



//        drive(1,1);
//        runtime.reset();
//        while(runtime.time()<1.0) {}

        drive(12);
        turnLeft();

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

    public void drive(double inches) {
        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);
        //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while(runtime.time()<0.15*inches) {}
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turnLeft() {
        leftDrive.setPower(0.4);

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setPower(0.4);

        runtime.reset();
        while(runtime.time()<0.15*3) {}
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        rightDrive.setDirection(DcMotor.Direction.FORWARD);

    }
    public void turnRight() {}

    public void dropOff(double inches) {
        drive(inches);
        drive(-inches);
    }
}
