package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "leftRedAuto")
public class leftRedAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor motor;
    private CRServo leftArm;
    private CRServo rightArm;
    private Servo claw;
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    SamplePipeline.PropPosition snapshotAnalysis = SamplePipeline.PropPosition.CENTER;

    private final double MOTOR_TICKS_PER_REV = 288;
    private final double MOTOR_CIRCUMFERENCE = 3.14;
    private final double MOTOR_TICKS_PER_INCH = MOTOR_TICKS_PER_REV/MOTOR_CIRCUMFERENCE;
    private final double TICKS_PER_REV = 28 * 20;
    private final double CIRCUMFERENCE = 10.99*1.1;
    private final double TICKS_PER_INCH = TICKS_PER_REV/CIRCUMFERENCE;

    @Override
    public void runOpMode() {
        camInit(1); // ** 1 for red, 2 for blue **

        motor = hardwareMap.get(DcMotor.class, "hexMotor");
        leftArm = hardwareMap.get(CRServo.class, "left");
        rightArm = hardwareMap.get(CRServo.class, "right");
        claw = hardwareMap.get(Servo.class, "claw");

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(.4);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftArm.setDirection(CRServo.Direction.FORWARD);
        rightArm.setDirection(CRServo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        leftArm.setPower(0);
        rightArm.setPower(0);

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


        closeClaw();
        //waitASec(.5);
        drive(24);
        switch(snapshotAnalysis) { //red-left
            case LEFT: {
                turnLeft(); //red left, left tp
                dropOff(4);
                turnRight();
                drive(26);
                turnLeft();
                waitASec(5);
                drive(-24*4);
                break;
            }
            case RIGHT: {
                turnRight(); //red left, right tp .375
                dropOff(9);
                turnLeft();
                drive(24);
                turnLeft();
                waitASec(5);
                drive(-24*4);
                break;
            }
            default: {
                dropOff(6); //red left, center tp
                turnLeft();
                drive(18);
                turnRight();
                drive(24);
                turnLeft();
                waitASec(5);
                drive(-24*4.5);
                break;
            }
        }
        openClaw();
    }

    //1 for red, 2 for blue
    public void camInit(int teamColor) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("webcam", "webcam", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //FtcDashboard.getInstance().startCameraStream(webcam, 30);
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
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0.3);
        rightDrive.setPower(0.3);

        leftDrive.setTargetPosition((int) (inches*TICKS_PER_INCH));
        rightDrive.setTargetPosition((int) (inches*TICKS_PER_INCH));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {}
    }

    public void turnLeft() {
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);

        waitASec(1.3);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void turnRight() {
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);

        waitASec(1.3);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void openClaw() {
        claw.setPosition(.6);
    }
    public void closeClaw() {
        claw.setPosition(.25);
    }

    public void dropOff(double inches) {
        drive(inches);
        //waitASec(.25);
        drive(-inches);
    }

    public void raiseArm() {
        leftArm.setPower(-.45);
        rightArm.setPower(-.45);
        waitASec(.75);
        leftArm.setPower(-.1);
        rightArm.setPower(-.1);
    }

    public void lowerArm() {
        leftArm.setPower(.2);
        rightArm.setPower(.2);
        waitASec(.75);
        leftArm.setPower(0);
        rightArm.setPower(0);
    }

    public void armForward() {
        motor.setTargetPosition((int) (4.1 * MOTOR_TICKS_PER_INCH));
        runtime.reset();
        while(motor.isBusy() && opModeIsActive()) {}
    }

    public void armBack() {
        motor.setTargetPosition(0);
        while(motor.isBusy() && opModeIsActive()) {}
    }

    public void waitASec(double seconds) {
        runtime.reset();
        while(runtime.time()<seconds && opModeIsActive()) {}
    }}
