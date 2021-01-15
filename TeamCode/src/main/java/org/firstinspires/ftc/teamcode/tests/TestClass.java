package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CameraType;
import org.firstinspires.ftc.teamcode.EasyOpenCVImportable;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name="Test Class", group="Test")
public class TestClass extends LinearOpMode {
    OpenCvCamera camera;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    public void runOpMode() {
        EasyOpenCVImportable easyOpenCVImportable = new EasyOpenCVImportable();

        easyOpenCVImportable.init(CameraType.PHONE, hardwareMap);

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
