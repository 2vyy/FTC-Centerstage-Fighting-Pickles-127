package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// a lot of this is inspired/stolen from https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/firstinspires/ftc/teamcode/WebcamExample.java
// the link has much more in depth explanations on what each line does and why

@Autonomous(name = "WIP_EasyOpenCV_Test")
public class WIP_EasyOpenCV_Test extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Servo leftSwiper;
    private Servo rightSwiper;
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    SamplePipeline.PropPosition snapshotAnalysis = SamplePipeline.PropPosition.CENTER;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    @Override
    public void runOpMode() {
        //leftblue demo
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("webcam", "webcam", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        pipeline = new SamplePipeline();
        pipeline.setTeamColor(2); // ** 1 for red, 2 for blue **
        webcam.setPipeline(pipeline);
        //webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // TODO: make sure resolution is highest that is supported by webcam
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                //0 for blue, 1 for red (maybe)
//                //webcam.setTeamColor(0);
//                snapshotAnalysis = pipeline.getAnalysis();
////                webcam.stopStreaming();
////                webcam.closeCameraDevice();
//                telemetry.addLine("Analysis: "+snapshotAnalysis);
            }
            @Override
            public void onError(int errorCode) {
                //called if cam cannot be opened
                snapshotAnalysis = SamplePipeline.PropPosition.CENTER;
                telemetry.addLine("Analysis: Failed\nError: "+errorCode+"\nDefaulting to Center");
            }
        });

        //cam code done :3

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftSwiper = hardwareMap.get(Servo.class, "leftSwiper");
        rightSwiper = hardwareMap.get(Servo.class, "rightSwiper");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); //probably reverse
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

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        snapshotAnalysis = pipeline.getAnalysis();

        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();
        switch (snapshotAnalysis) {
            case LEFT: {
                telemetry.addLine("LEFT");
            }
            case RIGHT: {
                telemetry.addLine("RIGHT");
            }
            default: {
                telemetry.addLine("CENTER");
            }
        }



        // Put initialization blocks here. NO NECESSARY CHANGES ABOVE HERE
//        while (opModeIsActive()) {
//            // Put loop blocks here.
//            telemetry.update();
//        }
    }

    public void openSwipers() {
        leftSwiper.setPosition(.5);
        rightSwiper.setPosition(.5);
    }
    public void closeSwipers() {
        leftSwiper.setPosition(0);
        rightSwiper.setPosition(0);
    }

    private void drive(double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            // Create target positions
            rightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_IN);

            // set target position
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            //switch to run to position mode
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the desiginated power
            leftDrive.setPower(power);
            rightDrive.setPower(power);

            // wait until both motors are no longer busy running to position //this entire part is cringe
            while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            }

            // set motor power back to 0
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }
}