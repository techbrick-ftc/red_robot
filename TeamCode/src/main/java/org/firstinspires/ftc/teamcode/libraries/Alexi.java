package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.PI;

public class Alexi {
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor rl = null;
    public DcMotor rr = null;

    public DcMotor intake = null;

    public DcMotorEx shooter = null;
    public Servo pusher = null;

    public DcMotorEx wobbleMotor = null;
    public Servo wobbleServo = null;

    public BNO055IMU imu = null;

    public DcMotor[] motors = null;
    public double[] angles = null;

    public void init(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        rl.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        motors = new DcMotor[]{fr, rr, rl, fl};
        angles = new double[]{PI / 4, 3 * PI / 4, 5 * PI / 4, 7 * PI / 4};

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        wobbleMotor.setTargetPosition(0);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);
    }
}
