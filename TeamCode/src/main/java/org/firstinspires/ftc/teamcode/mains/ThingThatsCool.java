package org.firstinspires.ftc.teamcode.mains;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.CameraTele;
import org.firstinspires.ftc.teamcode.libraries.FieldCentric;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import static java.lang.Math.PI;

@TeleOp(name="Driving", group="drive")
public class ThingThatsCool extends LinearOpMode implements TeleAuto {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor rl;
    private DcMotor rr;

    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private boolean shooterOn;


    private BNO055IMU imu;
    private T265Camera camera;

    private final CameraTele drive = new CameraTele();
    private final FieldCentric centric = new FieldCentric();

    private final Translation2d posA = new Translation2d(0, 30);
    private final Translation2d posB = new Translation2d(-18, 30);

    ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
    ScheduledFuture<?> pushEvent = null;

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");

        rl.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);

        if (camera == null) {
            camera = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        final DcMotor[] motors = {fr, rr, rl, fl};
        final double[] motorAngles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        centric.setUp(motors, motorAngles, imu);
        drive.setUp(motors, motorAngles, camera, imu, AxesReference.EXTRINSIC);

        startScheduler();

        boolean driving     = false;
        boolean turning     = false;
        boolean isMultiPart   = false;
        double movingX      = 0;
        double movingY      = 0;
        double movingTheta  = 0;
        int multiType    = 0;
        int multiPart    = 0;

        waitForStart();

        pusher.setPosition(-1);

        Gamepad prev1 = gamepad1;
        Gamepad prev2 = gamepad2;
        while (opModeIsActive()) {
            if (gamepad1.left_stick_x > 0.2     ||
                gamepad1.right_stick_x > 0.2    ||
                gamepad1.left_stick_y > 0.2     ||
                gamepad1.right_stick_y > 0.2) {
                driving = turning = false;
            }

            if (isMultiPart) {
                if (!drive.complete()) {
                    if ((multiType ^ multiPart) != 0) {
                        drive.goToPositionY(movingY);
                    } else {
                        drive.goToPositionX(movingX);
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
                centric.gyro(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
                centric.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }

            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                isMultiPart = true;
                multiType = 0;
                multiPart = 0;
                driving = turning = false;
            } else if (gamepad1.dpad_up || gamepad1.dpad_down) {
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

            if (gamepad1.x && !prev1.x) {
                movingTheta = PI/2;
                turning = true;
                driving = false;
                drive.newDrive();
            }

            if (gamepad2.b && !prev2.b) {
                if (!shooterOn) {
                    shooter.setPower(1);
                } else {
                    shooter.setPower(0);
                }
            }

            if (gamepad2.a && !prev2.b && pushEvent.isDone()) {
                pusher.setPosition(1);
                schedulePusher();
            }

            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intakePower);

            try {
                prev1.copy(gamepad1);
                prev2.copy(gamepad2);
            } catch (Exception ignore) {}
        }
    }

    private void startScheduler() {
        pushEvent = executorService.schedule(() -> {}, 0, TimeUnit.MILLISECONDS);
    }

    private void schedulePusher() {
        pushEvent = executorService.schedule(() -> {
            pusher.setPosition(0);
        }, 500, TimeUnit.MILLISECONDS);
    }
}