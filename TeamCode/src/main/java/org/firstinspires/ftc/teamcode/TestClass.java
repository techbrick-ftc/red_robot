package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="Test Class", group="Test")
public class TestClass extends LinearOpMode {
    OpenCvInternalCamera camera;

    public void runOpMode() {
        waitForStart();

        if (opModeIsActive()) {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

            try {
                EasyOpenCVImportable easyOpenCVImportable = new EasyOpenCVImportable(EasyOpenCVImportable.CameraType.WEBCAM, camera);
            } catch (Exception e) {
                e.printStackTrace();;
            }
        }
    }
}
