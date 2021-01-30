package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.CameraAuto;
import org.firstinspires.ftc.teamcode.mains.TeleAuto;

import static java.lang.Math.PI;

@Autonomous(name="Camera Drive", group="drive")
public class CameraTest extends LinearOpMode implements TeleAuto {
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor rl = null;
    DcMotor rr = null;
    BNO055IMU imu = null;
    T265Camera camera = null;
    CameraAuto drive = new CameraAuto();
    FtcDashboard dashboard = null;
    TelemetryPacket packet = null;

    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        camera = new T265Camera(new Transform2d(), 0.8, hardwareMap.appContext);
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        rl.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        packet.addLine("Hardware Done");
        dashboard.sendTelemetryPacket(packet);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);
        packet.addLine("IMU Done");
        dashboard.sendTelemetryPacket(packet);

        DcMotor[] motors = {fr, rr, rl, fl};
        double[] angles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        drive.setUp(motors, angles, camera, imu, AxesReference.EXTRINSIC, telemetry);
        packet.addLine("Set Up Done");
        dashboard.sendTelemetryPacket(packet);

        waitForStart();

        if (opModeIsActive()) {
            camera.start();
            drive.goToPosition(36, 0, 0.5, this);
            drive.goToRotation(PI, this);
            drive.goTo(0, 10, 3*PI/2, this);
        }
    }
}
