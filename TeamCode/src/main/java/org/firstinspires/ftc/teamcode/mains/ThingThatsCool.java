package org.firstinspires.ftc.teamcode.mains;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.CameraAuto;
import org.firstinspires.ftc.teamcode.libraries.CameraMain;
import org.firstinspires.ftc.teamcode.libraries.FieldCentric;

import static java.lang.Math.PI;

public class ThingThatsCool extends LinearOpMode implements TeleAuto {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor rl;
    private DcMotor rr;
    private BNO055IMU imu;
    private T265Camera camera;

    private final CameraAuto drive = new CameraAuto();
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

        waitForStart();

        Gamepad prev1 = gamepad1;
        while (opModeIsActive()) {
            centric.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            if(gamepad1.dpad_left & gamepad1.a) {}
            else if (gamepad1.a) {

            }

            try {
                prev1.copy(gamepad1);
            } catch (Exception ignore) {}
        }
    }
}