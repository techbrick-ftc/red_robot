package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class EasyOpenCVImportable {
    private OpenCvCamera webCamera;
    private OpenCvInternalCamera phoneCamera;

    private CameraType cameraType;

    private UltimateGoalDetectionPipeline pipeline;
    private boolean detecting;

    private double fieldTopLeftX = 181;
    private double fieldTopLeftY = 98;

    private double bottomTopLeftX = 181;
    private double bottomTopLeftY = 98;

    private double topTopLeftX = 181;
    private double topTopLeftY = 98;

    private int width = 90;
    private int height = 60;

    private int ringDiff = 129;
    private int fieldDiff = 123;

    public void init(CameraType cameraType, final HardwareMap hardwareMap) {
        this.cameraType = cameraType;
        if (cameraType.equals(CameraType.WEBCAM)) {
            initWebcam(hardwareMap, "Webcam 1");
        } else {
            initPhone(hardwareMap);
        }
    }

    public void initWebcam(final HardwareMap hardwareMap, final String webcamName) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        this.detecting = false;
    }

    public void initPhone(final HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        this.phoneCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
    }

    public void setFieldBox(double topLeftX, double topLeftY) {
        this.fieldTopLeftX = topLeftX;
        this.fieldTopLeftY = topLeftY;
    }

    public void setBottomBox(double topLeftX, double topLeftY) {
        this.bottomTopLeftX = topLeftX;
        this.bottomTopLeftY = topLeftY;
    }

    public void setTopBox(double topLeftX, double topLeftY) {
        this.topTopLeftX = topLeftX;
        this.topTopLeftY = topLeftY;
    }

    public void setSize(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public void setDifferences(int fieldDiff, int ringDiff) {
        this.fieldDiff = fieldDiff;
        this.ringDiff = ringDiff;
    }

    public void startDetection() {
        this.pipeline = new UltimateGoalDetectionPipeline();
        if (this.phoneCamera == null) {
            this.webCamera.setPipeline(this.pipeline);

            this.webCamera.openCameraDeviceAsync(() -> webCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
            this.detecting = true;
        } else {
            this.phoneCamera.setPipeline(this.pipeline);

            this.phoneCamera.openCameraDeviceAsync(() -> phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT));
        }
    }

    public void stopDetection() {
        if (this.cameraType.equals(CameraType.WEBCAM)) {
            this.webCamera.stopStreaming();
        } else {
            this.phoneCamera.stopStreaming();
        }
        this.detecting = false;
    }

    public OpenCvCamera getWebCamera() { return this.webCamera; }

    public OpenCvInternalCamera getPhoneCamera() { return this.phoneCamera; }

    public RingNumber getDetection() {
        return this.pipeline.number;
    }

    public boolean getDetecting() { return this.detecting; }

    public int getFieldAnalysis() { return this.pipeline.fieldAvg; }
    public int getBottomAnalysis() { return this.pipeline.bottomAvg; }
    public int getTopAnalysis() { return this.pipeline.topAvg; }

    public enum RingNumber {
        UNKNOWN,
        NONE,
        ONE,
        FOUR
    }

    private class UltimateGoalDetectionPipeline extends OpenCvPipeline {
        // Color constants
        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);
        final Scalar RED = new Scalar(255, 0, 0);

        // Core values for position and size
        //final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(fieldTopLeftX, fieldTopLeftY);
        final Point FIELD_TOPLEFT_ANCHOR_POINT = new Point(fieldTopLeftX, fieldTopLeftY);
        final Point BOTTOM_TOPLEFT_ANCHOR_POINT = new Point(bottomTopLeftX, bottomTopLeftY);
        final Point TOP_TOPLEFT_ANCHOR_POINT = new Point(topTopLeftX, topTopLeftY);

        /*final int REGION_WIDTH = fieldWidth;
        final int REGION_HEIGHT = fieldHeight;*/
        final int BOX_WIDTH = width;
        final int BOX_HEIGHT = height;

        final int FIELD_DIFF = fieldDiff;
        final int RING_DIFF = ringDiff;

        /*Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);*/

        Point field_pointA = new Point(
                FIELD_TOPLEFT_ANCHOR_POINT.x,
                FIELD_TOPLEFT_ANCHOR_POINT.y
        );
        Point field_pointB = new Point(
                FIELD_TOPLEFT_ANCHOR_POINT.x  + BOX_WIDTH,
                FIELD_TOPLEFT_ANCHOR_POINT.y + BOX_HEIGHT
        );

        Point bottom_pointA = new Point(
                BOTTOM_TOPLEFT_ANCHOR_POINT.x,
                BOTTOM_TOPLEFT_ANCHOR_POINT.y
        );
        Point bottom_pointB = new Point(
                BOTTOM_TOPLEFT_ANCHOR_POINT.x + BOX_WIDTH,
                BOTTOM_TOPLEFT_ANCHOR_POINT.y + BOX_HEIGHT
        );

        Point top_pointA = new Point(
                TOP_TOPLEFT_ANCHOR_POINT.x,
                TOP_TOPLEFT_ANCHOR_POINT.y
        );
        Point top_pointB = new Point(
                TOP_TOPLEFT_ANCHOR_POINT.x + BOX_WIDTH,
                TOP_TOPLEFT_ANCHOR_POINT.y + BOX_HEIGHT
        );

        // Working variables
        //Mat region1_Cb;
        Mat field_Cb;
        Mat bottom_Cb;
        Mat top_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        //int avg1;
        int fieldAvg;
        int bottomAvg;
        int topAvg;

        // Volatile since accessed by OpMode w/o synchronization
        private volatile RingNumber number = RingNumber.UNKNOWN;

        /*
            This take the RGB frame and converts it to YCrCb,
            and extracts the Cb channel to the "Cb" variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);
            field_Cb = Cb.submat(new Rect(field_pointA, field_pointB));
            bottom_Cb = Cb.submat(new Rect(bottom_pointA, bottom_pointB));
            top_Cb = Cb.submat(new Rect(top_pointA, top_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            fieldAvg = (int) Core.mean(field_Cb).val[0];
            bottomAvg = (int) Core.mean(bottom_Cb).val[0];
            topAvg = (int) Core.mean(top_Cb).val[0];

            if (fieldAvg == 0 ||
                bottomAvg == 0 ||
                topAvg == 0) {
                return input;
            }

            if (bottomAvg < fieldAvg + fieldDiff && bottomAvg > fieldAvg - fieldDiff) {
                number = RingNumber.NONE;
            } else {
                if (topAvg < bottomAvg + ringDiff && topAvg > bottomAvg - ringDiff) {
                    number = RingNumber.FOUR;
                } else {
                    number = RingNumber.ONE;
                }
            }

            // Field box
            Imgproc.rectangle(
                    input,
                    field_pointA,
                    field_pointB,
                    BLUE, 2
            );

            // Bottom box
            Imgproc.rectangle(
                    input,
                    bottom_pointA,
                    bottom_pointB,
                    RED, 2
            );

            // Top box
            Imgproc.rectangle(
                    input,
                    top_pointA,
                    top_pointB,
                    GREEN, 2
            );

            return input;
        }
    }
}

