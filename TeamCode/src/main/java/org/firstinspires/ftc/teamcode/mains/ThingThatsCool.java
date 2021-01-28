package org.firstinspires.ftc.teamcode.mains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private boolean shooterOn = false;
    private boolean shooterWait = false;
    private boolean pusherWait = false;

    private boolean a1Wait = false;
    private boolean b1Wait = false;
    private boolean x1Wait = false;


    private BNO055IMU imu;
    private T265Camera camera;

    private final CameraTele drive = new CameraTele();
    private final FieldCentric centric = new FieldCentric();

    private final Translation2d posA = new Translation2d(0, 30);
    private final Translation2d posB = new Translation2d(-18, 30);

    ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
    ScheduledFuture<?> pushEvent = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

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
            //camera = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
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

        pusher.setPosition(1);
        //camera.start();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_x > 0.2     ||
                gamepad1.right_stick_x > 0.2    ||
                gamepad1.left_stick_y > 0.2     ||
                gamepad1.right_stick_y > 0.2) {
                driving = turning = false;
            }

            if (isMultiPart && multiPart > 2) {
                if (!drive.complete()) {
                    if ((multiType ^ multiPart) != 0) {
                        drive.goToPositionY(movingY);
                    } else {
                        drive.goToPositionX(movingX);
                    }
                } else {
                    if (multiPart > 2) {
                        multiPart++;
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

            if (gamepad1.a && !a1Wait) {
                movingX = posA.getX();
                movingY = posA.getY();
                driving = true;
                turning = false;
                a1Wait = true;
                drive.newDrive();
            } else {
                a1Wait = false;
            }

            if (gamepad1.b && !b1Wait) {
                movingX = posB.getX();
                movingY = posB.getY();
                driving = true;
                turning = false;
                b1Wait = true;
                drive.newDrive();
            } else {
                b1Wait = false;
            }

            if (gamepad1.x && !x1Wait) {
                movingTheta = PI/2;
                turning = true;
                driving = false;
                x1Wait = true;
                drive.newDrive();
            } else {
                x1Wait = false;
            }

            if (gamepad1.back) {
                centric.resetAngle();
            }

            if (gamepad2.b && !shooterWait) {
                if (!shooterOn) {
                    shooter.setPower(1);
                    shooterOn = true;
                } else {
                    shooter.setPower(0);
                    shooterOn = false;
                }
                shooterWait = true;
            } else if (!gamepad2.b) {
                shooterWait = false;
            }

            telemetry.addData("Gamepad 2 B", gamepad2.b);
            telemetry.addData("Shooter Wait", shooterWait);
            telemetry.update();

            if (gamepad2.a && !pusherWait && pushEvent.isDone()) {
                pusher.setPosition(0);
                pusherWait = true;
                schedulePusher();
            } else {
                pusherWait = false;
            }

            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intakePower);
        }
    }

    private void startScheduler() {
        pushEvent = executorService.schedule(() -> {}, 0, TimeUnit.MILLISECONDS);
    }

    private void schedulePusher() {
        pushEvent = executorService.schedule(() -> pusher.setPosition(1), 1000, TimeUnit.MILLISECONDS);
    }
}