package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="Test Class", group="Test")
public class TestClass extends LinearOpMode {
    OpenCvCamera camera;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    public void runOpMode() {
        EasyOpenCVImportable easyOpenCVImportable = new EasyOpenCVImportable();

        easyOpenCVImportable.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        waitForStart();

        while (opModeIsActive()) {
            easyOpenCVImportable.startDetection();
            FtcDashboard.getInstance().startCameraStream(easyOpenCVImportable.getWebCamera(), 0);

            while (opModeIsActive() && !isStopRequested()) {
                packet.put("Detecting", easyOpenCVImportable.getDetecting());
                packet.put("Detected", easyOpenCVImportable.getDetection());
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}
