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

import java.util.Vector;

public class EasyOpenCVImportable {
    private OpenCvCamera webCamera;
    private OpenCvInternalCamera phoneCamera;

    private CameraType cameraType;

    private UltimateGoalDetectionPipeline pipeline;
    private boolean detecting;

    private double topLeftX = 181;
    private double topLeftY = 98;
    private int width = 90;
    private int height = 60;

    private int fourThresh = 129;
    private int oneThresh = 123;

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

    public void setBox(double topLeftX, double topLeftY, int width, int height) {
        this.topLeftX = topLeftX;
        this.topLeftY = topLeftY;
        this.width = width;
        this.height = height;
    }

    public void setThresholds(int oneThresh, int fourThresh) {
        this.oneThresh = oneThresh;
        this.fourThresh = fourThresh;
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
        int avg1 = this.pipeline.avg1;
        if (avg1 > 0) {
            return this.pipeline.number;
        } else {
            return RingNumber.UNKNOWN;
        }
    }

    public boolean getDetecting() { return this.detecting; }

    public int getAnalysis() { return this.pipeline.avg1; }

    public int getHValue() { return this.pipeline.hAvg; }
    public int getSValue() { return this.pipeline.sAvg; }
    public int getVValue() { return this.pipeline.vAvg; }

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

        // Core values for position and size
        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(topLeftX, topLeftY);

        final int REGION_WIDTH = width;
        final int REGION_HEIGHT = height;

        final int FOUR_RING_THRESHOLD = fourThresh;
        final int ONE_RING_THRESHOLD = oneThresh;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Working variables
        Mat region1_Cb;
        Mat region1_h;
        Mat region1_s;
        Mat region1_v;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat HSV = new Mat();
        Vector<Mat> hsv_planes = new Vector<>();
        int avg1;
        int hAvg;
        int sAvg;
        int vAvg;

        // Volatile since accessed by OpMode w/o synchronization
        private volatile RingNumber number = RingNumber.FOUR;

        /*
            This take the RGB frame and converts it to YCrCb,
            and extracts the Cb channel to the "Cb" variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        void inputToHSV(Mat input){
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Core.split(HSV, hsv_planes);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToHSV(firstFrame);
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            //region1_h = hsv_planes.get(0).submat(new Rect(region1_pointA, region1_pointB));
            //region1_s = hsv_planes.get(1).submat(new Rect(region1_pointA, region1_pointB));
            //region1_v = hsv_planes.get(2).submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToHSV(input);

            hAvg = (int) Core.mean(region1_h).val[0];
            sAvg = (int) Core.mean(region1_s).val[0];
            vAvg = (int) Core.mean(region1_v).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE, 2);

            return input;
        }
    }
}

