package org.firstinspires.ftc.teamcode.libraries;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import static java.lang.Math.PI;

public class Alexi {
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor rl = null;
    public DcMotor rr = null;

    public DcMotor intake = null;

    public DcMotor shooter = null;
    public Servo pusher = null;

    public DcMotor wobbleMotor = null;
    public Servo wobbleServo = null;

    public BNO055IMU imu = null;

    public T265Camera camera = null;

    public DcMotor[] motors = null;
    public double[] angles = null;

    public void init(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        motors = new DcMotor[]{fl, fr, rl, rr};
        angles = new double[]{PI / 4, 3 * PI / 4, 5 * PI / 4, 7 * PI / 4};

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");

        wobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);

        camera = new T265Camera(new Transform2d(), 0.8, hardwareMap.appContext);
    }
}
