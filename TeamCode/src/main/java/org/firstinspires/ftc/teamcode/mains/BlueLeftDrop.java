package org.firstinspires.ftc.teamcode.mains;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.Alexi;
import org.firstinspires.ftc.teamcode.libraries.AutonomousFunctions;
import org.firstinspires.ftc.teamcode.libraries.CameraAuto;
import org.firstinspires.ftc.teamcode.libraries.CameraType;
import org.firstinspires.ftc.teamcode.libraries.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.libraries.TeleAuto;

@Autonomous(name="Blue Left Drop", group="Blue")
public class BlueLeftDrop extends LinearOpMode implements TeleAuto {
    Alexi robot = new Alexi();
    CameraAuto drive = new CameraAuto();
    AutonomousFunctions auto = new AutonomousFunctions(robot, this);
    EasyOpenCVImportable openCV = new EasyOpenCVImportable();
    ElapsedTime et = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.wobbleServo.setPosition(-1);

        drive.setUp(robot.motors, robot.angles, robot.camera, robot.imu, AxesReference.EXTRINSIC, telemetry);
        drive.setPose(new Pose2d(-56, 46, new Rotation2d()));

        openCV.init(CameraType.WEBCAM, hardwareMap);
        openCV.setBox(250, 140, 70, 50);
        openCV.setThresholds(135, 148);

        waitForStart();

        if (opModeIsActive()) {
            robot.camera.start();
            openCV.startDetection();
            while (openCV.getDetection().equals(EasyOpenCVImportable.RingNumber.UNKNOWN)) { idle(); }
            et.reset();
            while (et.seconds() < 1) { idle(); }
            telemetry.addData("Detected", openCV.getDetection());
            telemetry.update();
            if (openCV.getDetection().equals(EasyOpenCVImportable.RingNumber.NONE)) {
                openCV.stopDetection();
                drive.goTo(10, 45, 0, this);
                auto.armDownAndDrop();
                drive.goToPosition(12, 0, this);
            } else if (openCV.getDetection().equals(EasyOpenCVImportable.RingNumber.ONE)) {
                openCV.stopDetection();
                drive.goToPosition(-22, 10, this);
                drive.goTo(36, 24, 0, this);
                auto.armDownAndDrop();
                drive.goToPosition(12, 12, this);
            } else if (openCV.getDetection().equals(EasyOpenCVImportable.RingNumber.FOUR)) {
                openCV.stopDetection();
                drive.goToPosition(-22, 10, this);
                drive.goTo(58, 45, 0, this);
                auto.armDownAndDrop();
                drive.goToPosition(12, 0, this);
            }
        }
    }
}
