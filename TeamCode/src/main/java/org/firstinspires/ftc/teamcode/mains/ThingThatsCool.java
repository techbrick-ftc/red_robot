package org.firstinspires.ftc.teamcode.mains;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.CameraTele;
import org.firstinspires.ftc.teamcode.libraries.FieldCentric;

import static java.lang.Math.PI;

public class ThingThatsCool extends LinearOpMode implements TeleAuto {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor rl;
    private DcMotor rr;
    private BNO055IMU imu;
    private T265Camera camera;

    private final CameraTele drive = new CameraTele();
    private final FieldCentric centric = new FieldCentric();

    private final Translation2d posA = new Translation2d(0, 30);
    private final Translation2d posB = new Translation2d(-18, 30);

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        if (camera == null) {
            camera = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        final DcMotor[] motors = {fr, rr, rl, fl};
        final double[] motorAngles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        centric.setUp(motors, motorAngles, imu);
        drive.setUp(motors, motorAngles, camera, imu, AxesReference.EXTRINSIC);

        boolean driving     = false;
        boolean turning     = false;
        boolean isMultiPart   = false;
        double movingX      = 0;
        double movingY      = 0;
        double movingTheta  = 0;
        int multiType    = 0;
        int multiPart    = 0;

        waitForStart();

        Gamepad prev1 = gamepad1;
        while (opModeIsActive()) {
            if (gamepad1.left_stick_x > 0.2     ||
                gamepad1.right_stick_x > 0.2    ||
                gamepad1.left_stick_y > 0.2     ||
                gamepad1.right_stick_y > 0.2) {
                driving = turning = false;
            }

            if (isMultiPart) {
                if (!drive.complete()) {
                    if (multiType == 0) {
                        switch (multiPart) {
                            case 0:
                                drive.goToPositionX(movingX);
                                break;
                            case 1:
                                drive.goToPositionY(movingY);
                                break;
                            default:
                        }
                    } else {
                        switch (multiPart) {
                            case 0:
                                drive.goToPositionY(movingY);
                                break;
                            case 1:
                                drive.goToPositionX(movingX);
                                break;
                            default:
                        }
                    }
                } else {
                    if (multiPart == 1) {
                        isMultiPart = false;
                    } else {
                        multiPart += 1;
                        drive.newDrive();
                    }
                }
            } else if (drive.complete()) {
                driving = turning = false;
            }

            if (driving && turning) {
                drive.goTo(movingX, movingY, movingTheta);
            } else if (driving) {
                drive.goToPosition(movingX, movingY);
            } else if (turning) {
                drive.goToRotation(movingTheta);
            } else {
                centric.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }

            if ((gamepad1.dpad_right && !prev1.dpad_right) || (gamepad1.dpad_left && !prev1.dpad_left)) {
                isMultiPart = true;
                multiType = 0;
                multiPart = 0;
                driving = turning = false;
            } else if ((gamepad1.dpad_up && !prev1.dpad_up) || (gamepad1.dpad_down && !prev1.dpad_down)) {
                isMultiPart = true;
                multiType = 1;
                multiPart = 0;
                driving = turning = false;
            } else {
                isMultiPart = false;
            }

            if (gamepad1.a && !prev1.a) {
                movingX = 0;
                movingY = 20;
                driving = true;
                turning = false;
                drive.newDrive();
            }

            if (gamepad1.b && !prev1.b) {
                movingX = -20;
                movingY = 20;
                driving = true;
                turning = false;
                drive.newDrive();
            }

            if (gamepad1.x) {
                movingTheta = PI/2;
                turning = true;
                driving = false;
                drive.newDrive();
            }

            try {
                prev1.copy(gamepad1);
            } catch (Exception ignore) {}
        }
    }
}