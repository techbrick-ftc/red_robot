package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.libraries.CameraType;
import org.firstinspires.ftc.teamcode.libraries.EasyOpenCVImportable;

@Autonomous(name="Webcam Test", group="test")
//@Disabled
public class WebcamTest extends LinearOpMode {
    EasyOpenCVImportable openCV = new EasyOpenCVImportable();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        openCV.init(CameraType.WEBCAM, hardwareMap);
        openCV.setFieldBox(RobotConfig.fieldX, RobotConfig.fieldY);
        openCV.setBottomBox(RobotConfig.bottomX, RobotConfig.bottomY);
        openCV.setTopBox(RobotConfig.topX, RobotConfig.topY);
        openCV.setSize(RobotConfig.boxWidth, RobotConfig.boxHeight);
        openCV.setDifferences(RobotConfig.fieldDiff, RobotConfig.ringDiff);

        waitForStart();

        openCV.startDetection();
        dashboard.startCameraStream(openCV.getWebCamera(), 0);

        while (opModeIsActive()) {
            packet.put("Field", openCV.getFieldAnalysis());
            packet.put("Bottom", openCV.getBottomAnalysis());
            packet.put("Top", openCV.getTopAnalysis());
            packet.put("Detected", openCV.getDetection());
            dashboard.sendTelemetryPacket(packet);
        }

        if (openCV.getDetecting()) {
            dashboard.stopCameraStream();
            openCV.stopDetection();
        }
    }
}
