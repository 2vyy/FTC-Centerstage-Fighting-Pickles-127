package org.firstinspires.ftc.teamcode;

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
    int isRed = 0;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    public DcMotor[] motors = new DcMotor[]{leftFront, leftRear, rightFront, rightRear};
    private DcMotor slides;
    private Servo right;
    private Servo left;
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    SamplePipeline.PropPosition snapshotAnalysis = SamplePipeline.PropPosition.CENTER;

    @Override
    public void runOpMode() {
        //leftblue demo
        camInit();
        waitForStart();
        // Put initialization blocks here. NO NECESSARY CHANGES ABOVE HERE
        if (opModeIsActive()) {
            // PUT AUTONOMOUS INSTRUCTIONS HERE
            closeClaw();
            Move_Forward_Backward(24+3,1);
            switch (snapshotAnalysis) {
                case LEFT: {
                    Turning(-90);
                    openClaw();
                    Turning(-180);
                }
                case RIGHT: {
                    Turning(90);
                    openClaw();
                }
                default: {
                    openClaw();
                    Turning(90);
                }
            }
            Move_Left_Right(-24);
            Move_Forward_Backward(-24*2,1);
        }
        while (opModeIsActive()) {
            // Put loop blocks here.
            telemetry.update();
        }
    }

    public void camInit() {
        // TODO: when webcam is connected, set id to what it is in app config
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // TODO: make sure resolution is highest that is supported by webcam
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                //0 for blue, 1 for red (maybe)
                pipeline.setTeamColor(isRed);
                snapshotAnalysis = pipeline.getAnalysis();
                webcam.stopStreaming();
                webcam.closeCameraDevice();
                telemetry.addLine("Analysis: "+snapshotAnalysis);
            }
            @Override
            public void onError(int errorCode) {
                //called if cam cannot be opened
                snapshotAnalysis = SamplePipeline.PropPosition.CENTER;
                telemetry.addLine("Analysis: Failed\nError: "+errorCode+"\nDefaulting to Center");
            }
        });
        telemetry.update();
    }
    private void Move_Forward_Backward(int Distance, double Speed) {
        Reset();
        for (DcMotor motor : motors) {
            motor.setTargetPosition(Distance*120);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Speed);
        }
        while (motors[0].isBusy() || motors[3].isBusy()) {
            sleep(100);
        }
    }

    private void Turning(double Target_Degrees) {
        Reset();
        motors[0].setTargetPosition((int) (Target_Degrees * -26.44));
        motors[1].setTargetPosition((int) (Target_Degrees * -26.44));
        motors[2].setTargetPosition((int) (Target_Degrees * 26.44));
        motors[3].setTargetPosition((int) (Target_Degrees * 26.44));
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        while (motors[0].isBusy() || motors[3].isBusy()) {
            sleep(100);
        }
    }

    // inches
    private void Move_Left_Right(int Distance) {
        Reset();
        motors[0].setTargetPosition(Distance * 144);
        motors[2].setTargetPosition(Distance * -144);
        motors[1].setTargetPosition(Distance * -144);
        motors[3].setTargetPosition(Distance * 144);
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.8);
        }
        while (motors[0].isBusy() || motors[3].isBusy()) {
            sleep(100);
        }
    }

    //0 to idk like 9000 ish?
    private void SlideMovement(int Target) {
        Reset();
        slides.setTargetPosition(Target);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
        while (slides.isBusy()) {
            sleep(100);
        }
    }
    private void openClaw() {
        right.setPosition(0.275);
        left.setPosition(0.275);
    }
    private void closeClaw() {
        right.setPosition(0.5);
        left.setPosition(0.5);
    }
    private void Reset() {
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}