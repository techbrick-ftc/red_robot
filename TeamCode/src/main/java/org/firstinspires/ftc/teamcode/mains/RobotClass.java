package org.firstinspires.ftc.teamcode.mains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.FieldCentric;

import static java.lang.Math.PI;

@TeleOp(name="Robo Drive Class", group="Test")
public class RobotClass extends LinearOpMode {
    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor rlMotor;
    DcMotor rrMotor;

    BNO055IMU imu;

    DcMotor intake;

    FieldCentric drive = new FieldCentric();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");

        rlMotor.setDirection(DcMotor.Direction.REVERSE);
        rrMotor.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile  = "BNO055IMUCalibration.json";
        params.loggingEnabled       = true;
        params.loggingTag           = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(params);

        DcMotor[] motors = {frMotor, rrMotor, rlMotor, flMotor};
        double[] motorAngles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        drive.setUp(motors, motorAngles, imu);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.gyro(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
            drive.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            double intakeSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intakeSpeed);
        }
    }
}
