package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private Servo leftSwiper;
    private Servo rightSwiper;
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    SamplePipeline.PropPosition snapshotAnalysis = SamplePipeline.PropPosition.CENTER;

    private final int TICKS_PER_REV = 28 * 3; //assuming its a 3:1 gear ratio, idk
    private final double CIRCUMFERENCE = 12.57;
    private final double TICKS_PER_INCH = TICKS_PER_REV/CIRCUMFERENCE;

    @Override
    public void runOpMode() {
        camInit(2);

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftSwiper = hardwareMap.get(Servo.class, "leftSwiper");
        rightSwiper = hardwareMap.get(Servo.class, "rightSwiper");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftSwiper.setDirection(Servo.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightSwiper.setDirection(Servo.Direction.REVERSE);

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

        switch (snapshotAnalysis) {
            case LEFT: {

            }
            case RIGHT: {

            }
            default: {

            }
        }
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

    public void drive(double power, double leftInches, double rightInches) {
        //leftDrive.setTargetPosition((int) (leftInches*DRIVE_TICKS_PER_INCH));
        //rightDrive.setTargetPosition((int) (rightInches*DRIVE_TICKS_PER_INCH));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition( (int) (leftInches*TICKS_PER_INCH));
        rightDrive.setTargetPosition( (int) (rightInches*TICKS_PER_INCH));

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
        }

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
