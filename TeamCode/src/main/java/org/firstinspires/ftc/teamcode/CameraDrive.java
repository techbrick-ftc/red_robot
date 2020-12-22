package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

public class CameraDrive {
    private DcMotor[] motors;
    private double[] motorSpeeds;
    private double[] angles;
    private BNO055IMU imu;
    private FtcDashboard ftcDashboard = FtcDashboard.getInstance();

    public void setUp(DcMotor[] motors, double[] angles, BNO055IMU imu) {
        if (motors.length != angles.length) {
            throw new RuntimeException("Motor array length and angle array length are not the same! Check your code!");
        }
        this.motors = motors;
        this.angles = angles;
        this.imu = imu;
    }

    public void goToPosition(T265Camera camera, double moveX, double moveY, TeleAuto callback) {
        goToPosition(camera, moveX, moveY, 1, callback);
    }

    public void goToPosition(T265Camera camera, double moveX, double moveY, double speed, TeleAuto callback) {
        while (callback.opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            T265Camera.CameraUpdate up = camera.getLastReceivedCameraUpdate();
            if (up == null) return;

            // We divide by 0.0254 to convert meters to inches
            Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            field.strokeCircle(translation.getX(), translation.getY(), 9);
            double arrowX = rotation.getCos() * 9, arrowY = rotation.getSin() * 9;
            double x1 = translation.getX() + arrowX / 2, y1 = translation.getY() + arrowY / 2;
            double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            speed = Math.min(-1, Math.max(speed, 1));

            double currentX = translation.getX();
            double currentY = translation.getY();
            packet.put("currentX", currentX);
            packet.put("currentY", currentY);

            double deltaX = moveX - currentX;
            double deltaY = moveY - currentY;
            packet.put("deltaX", deltaX);
            packet.put("deltaY", deltaY);

            if (deltaX < 1) {
                deltaX = 0;
            }
            if (deltaY < 1) {
                deltaY = 0;
            }
            if (deltaX == 0 && deltaY == 0) {
                break;
            }

            double theta = Math.atan2(deltaY, deltaX);
            packet.put("theta", theta);

            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(Math.sin(angles[i] - theta) * speed);
                packet.put("Motor " + i, Math.sin(angles[i] - theta) * speed);
            }

            ftcDashboard.sendTelemetryPacket(packet);
        }
    }
}

interface TeleAuto {
    boolean opModeIsActive();
}
