package org.firstinspires.ftc.teamcode.mains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.FieldCentric;
import org.firstinspires.ftc.teamcode.libraries.TeleAuto;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import static java.lang.Math.PI;

@TeleOp(name="Driving", group="drive")
public class MainDrive extends LinearOpMode implements TeleAuto {
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

    private DcMotorEx wobbleMotor;
    private Servo wobbleServo;
    private int[] wobblePositions = {0, -734, -1368, -1641};
    private double wobbleSpeed = 0;
    private int wobblePosition = 0;
    private boolean wobbleWait = false;

    private boolean a1Wait = false;
    private boolean b1Wait = false;
    private boolean x1Wait = false;


    private BNO055IMU imu;

    private final FieldCentric centric = new FieldCentric();

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

        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rl.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);

        final DcMotor[] motors = {fr, rr, rl, fl};
        final double[] motorAngles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        centric.setUp(motors, motorAngles, imu);

        startScheduler();

        waitForStart();

        pusher.setPosition(1);

        while (opModeIsActive()) {
            if (gamepad1.back) {
                centric.resetAngle();
            }

            centric.gyro(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
            centric.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            if (gamepad2.y && !shooterWait) {
                if (!shooterOn) {
                    shooter.setPower(0.8);
                    shooterOn = true;
                } else {
                    shooter.setPower(0);
                    shooterOn = false;
                }
                shooterWait = true;
            } else if (!gamepad2.y) {
                shooterWait = false;
            }

            if (gamepad2.a && !pusherWait && pushEvent.isDone()) {
                pusher.setPosition(0.1);
                pusherWait = true;
                schedulePusher();
            } else {
                pusherWait = false;
            }

            wobbleMotor.setPower(gamepad2.left_stick_x);

            if (gamepad2.x) {
                wobbleServo.setPosition(-1);
            } else if (gamepad2.b) {
                wobbleServo.setPosition(1);
            }

            if (gamepad2.dpad_right && wobblePosition < 3 && !wobbleWait) {
                wobblePosition++;
                wobbleWait = true;
            } else if (gamepad2.dpad_left && wobblePosition > 0 && !wobbleWait) {
                wobblePosition--;
                wobbleWait = true;
            } else if (!gamepad2.dpad_right && !gamepad2.dpad_left) { wobbleWait = false; }
            wobbleMotor.setTargetPosition(wobblePositions[wobblePosition]);
            if (wobbleMotor.isBusy()) {
                wobbleMotor.setVelocity(800);
            } else {
                wobbleMotor.setVelocity(0);
            }

            telemetry.addData("Target", wobbleMotor.getTargetPosition());
            telemetry.addData("Position", wobblePosition);
            telemetry.addData("Wobble Speed", wobbleSpeed);
            telemetry.addData("Is Busy", wobbleMotor.isBusy());
            telemetry.addData("Gamepad2 Dpad Right", gamepad2.dpad_right);
            telemetry.addData("Wobble Wait", wobbleWait);
            telemetry.update();

            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intakePower);
        }
    }

    private void startScheduler() {
        pushEvent = executorService.schedule(() -> {}, 0, TimeUnit.MILLISECONDS);
    }

    private void schedulePusher() {
        pushEvent = executorService.schedule(() -> pusher.setPosition(0.9), 1000, TimeUnit.MILLISECONDS);
    }
}