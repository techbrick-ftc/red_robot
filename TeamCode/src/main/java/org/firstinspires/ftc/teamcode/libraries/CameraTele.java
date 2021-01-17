package org.firstinspires.ftc.teamcode.libraries;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class CameraTele {
    private final CameraMain MAIN = new CameraMain();

    private boolean complete = false;

    /**
     * Sets up internal variables for driving
     * @param motors Array of motors
     * @param angles Array of angles (referenced in same order as motors)
     * @param camera Camera
     * @param imu IMU
     * @param axesReference Axes reference for getting the IMU angles
     */
    public void setUp(DcMotor[] motors, double[] angles, T265Camera camera, BNO055IMU imu, AxesReference axesReference) {
        this.MAIN.setUpInternal(motors, angles, camera, imu, axesReference, null);
    }

    /**
     * Sets up internal variables for driving
     * @param motors Array of motors
     * @param angles Array of angles (referenced in same order as motors)
     * @param camera Camera
     * @param imu IMU
     * @param axesReference Axes reference for getting the IMU angles
     * @param telemetry (Optional) Telemetry from OpMode for seeing details
     */
    public void setUp(DcMotor[] motors, double[] angles, T265Camera camera, BNO055IMU imu, AxesReference axesReference, Telemetry telemetry) {
        this.MAIN.setUpInternal(motors, angles, camera, imu, axesReference, telemetry);
    }

    public void goToRotation(double theta) {
        goToRotation(theta, 1);
    }

    public void goToRotation(double theta, double speed) {
        Translation2d currentPos = this.MAIN.getPosition();
        double currentX = currentPos.getX();
        double currentY = currentPos.getY();
        complete = this.MAIN.goToInternal(currentX, currentY, theta, speed);
    }

    public void goToPositionX (double moveX) {
        goToPositionX(moveX, 1);
    }

    public void goToPositionX (double moveX, double speed) {
        Translation2d currentPos = this.MAIN.getPosition();
        goToPosition(moveX, currentPos.getY(), speed);
    }

    public void goToPositionY (double moveY) {
        goToPositionY(moveY, 1);
    }

    public void goToPositionY (double moveY, double speed) {
        Translation2d currentPos = this.MAIN.getPosition();
        goToPosition(currentPos.getX(), moveY, speed);
    }

    public void goToPosition(double moveX, double moveY) {
        goToPosition(moveX, moveY, 1);
    }

    public void goToPosition(double moveX, double moveY, double speed) {
        double angle = this.MAIN.getRotation().firstAngle;
        complete = this.MAIN.goToInternal(moveX, moveY, angle, speed);
    }

    public void goTo(double moveX, double moveY, double theta) {
        complete = this.MAIN.goToInternal(moveX, moveY, theta, 1);
    }

    public void goTo(double moveX, double moveY, double theta, double speed) {
        complete = this.MAIN.goToInternal(moveX, moveY, theta, speed);
    }

    public void newDrive() {
        complete = false;
    }

    public boolean complete() {
        return complete;
    }
}
