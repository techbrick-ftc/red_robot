package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.CameraType;
import org.firstinspires.ftc.teamcode.libraries.EasyOpenCVImportable;

@Autonomous(name="Webcam Test", group="test")
public class WebcamTest extends LinearOpMode {
    EasyOpenCVImportable openCV = new EasyOpenCVImportable();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        openCV.init(CameraType.WEBCAM, hardwareMap);
        openCV.setBox(80, 110, 70, 50);
        openCV.setThresholds(135, 148);

        waitForStart();

        openCV.startDetection();
        dashboard.startCameraStream(openCV.getWebCamera(), 0);

        while (opModeIsActive()) {
            packet.put("Detecting", openCV.getDetection());
            packet.put("Analysis", openCV.getAnalysis());
            dashboard.sendTelemetryPacket(packet);
        }

        if (openCV.getDetecting()) {
            dashboard.stopCameraStream();
            openCV.stopDetection();
        }
    }
}
