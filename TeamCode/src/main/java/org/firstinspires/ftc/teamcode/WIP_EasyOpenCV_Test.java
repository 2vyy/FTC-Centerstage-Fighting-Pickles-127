package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


// a lot of this is inspired/stolen from https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/firstinspires/ftc/teamcode/WebcamExample.java
// the link has much more in depth explanations on what each line does and why

@Autonomous(name = "WIP_EasyOpenCV_Test")
public class WIP_EasyOpenCV_Test extends LinearOpMode {
    OpenCvWebcam webcam;
    WIP_EasyOpenCV_Test.SamplePipeline pipeline;
    WIP_EasyOpenCV_Test.SamplePipeline.PropPosition snapshotAnalysis = SamplePipeline.PropPosition.CENTER;

    @Override
    public void runOpMode() {
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
            }
            @Override
            public void onError(int errorCode) {
                //called if cam cannot be opened
            }
        });
        snapshotAnalysis = pipeline.getAnalysis();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        telemetry.addLine("Analysis: "+snapshotAnalysis);
        telemetry.update();
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

    public static class SamplePipeline extends OpenCvPipeline {
        public enum PropPosition {LEFT, CENTER, RIGHT}
        private volatile PropPosition position = PropPosition.CENTER;
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //TODO: find exact numbers for these once cam is in place, also update resolution used if changed
        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(320*.25,240*.5);
        final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(320*.5,240*.25);
        final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(320*.75,240*.5);
        final int REGION_WIDTH = 20;
        final int REGION_HEIGHT = 20;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Mat region1, region2, region3;
        Mat YCrCb = new Mat();
        Mat channel = new Mat();
        int avg1, avg2, avg3;

        int teamColor;

        //set team color to know whether to use cr or cb, probably stupid to set it separately but whatever
        //blue is 0, red is (probably) 1
        public void setTeamColor(int color) {
            teamColor = color;
        }

        //using YCbCr bc it separates color from brightness
        public void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, channel, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            //assigning rectangular subregions to each region
            region1 = channel.submat(new Rect(region1_pointA, region1_pointB));
            region2 = channel.submat(new Rect(region2_pointA, region2_pointB));
            region3 = channel.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1).val[teamColor];
            avg2 = (int) Core.mean(region2).val[teamColor];
            avg3 = (int) Core.mean(region3).val[teamColor];

            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);
            Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE, 2);

            int max = Math.max(Math.max(avg1, avg2), avg3);

            if(max == avg1) {
                position = PropPosition.LEFT;
                Imgproc.rectangle(input, region1_pointA, region1_pointB, GREEN, -1);
            } else if (max == avg2) { //maybe change to have center be default
                position = PropPosition.CENTER;
                Imgproc.rectangle(input, region2_pointA, region2_pointB, GREEN, -1);
            } else {
                position = PropPosition.RIGHT;
                Imgproc.rectangle(input, region3_pointA, region3_pointB, GREEN, -1);
            }

            return input;
        }

        public PropPosition getAnalysis() {
            return position;
        }
    }
}