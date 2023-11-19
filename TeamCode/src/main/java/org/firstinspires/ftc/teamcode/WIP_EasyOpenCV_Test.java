package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    int isLeft = 1;
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    SamplePipeline.PropPosition snapshotAnalysis = SamplePipeline.PropPosition.CENTER;

    @Override
    public void runOpMode() {
        camInit();
        waitForStart();
        // Put initialization blocks here. NO NECESSARY CHANGES ABOVE HERE
        if (opModeIsActive()) {
            // PUT AUTONOMOUS INSTRUCTIONS HERE
            switch (snapshotAnalysis) {
                case LEFT: {

                }
                case RIGHT: {

                }
                case CENTER: {

                }
            }
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
}