package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import static java.lang.Math.PI;

@TeleOp(name="Camera Drive", group="drive")
public class CameraDriveTest extends LinearOpMode implements TeleAuto {
    DcMotor flMotor = null;
    DcMotor frMotor = null;
    DcMotor rlMotor = null;
    DcMotor rrMotor = null;
    BNO055IMU imu = null;
    T265Camera camera = null;
    CameraDrive drive = new CameraDrive();
    FtcDashboard dashboard = null;
    TelemetryPacket packet = null;

    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        if (camera == null) {
            camera = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        packet.addLine("Camera Done");
        dashboard.sendTelemetryPacket(packet);

        waitForStart();

        if (opModeIsActive()) {
            flMotor = hardwareMap.get(DcMotor.class, "flMotor");
            frMotor = hardwareMap.get(DcMotor.class, "frMotor");
            rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
            rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");
            packet.addLine("Hardware Done");
            dashboard.sendTelemetryPacket(packet);

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imu.initialize(params);
            packet.addLine("IMU Done");
            dashboard.sendTelemetryPacket(packet);

            DcMotor[] motors = {flMotor, rlMotor, rrMotor, frMotor};
            double[] angles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

            drive.setUp(motors, angles, imu);
            packet.addLine("Set Up Done");
            dashboard.sendTelemetryPacket(packet);

            camera.start();


            drive.goToPosition(camera, 12, 12, 0.2, this);

            camera.stop();
        }
    }
}
