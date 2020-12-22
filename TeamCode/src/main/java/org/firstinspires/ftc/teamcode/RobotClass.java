package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Robo Test Class", group="Test")
public class RobotClass extends LinearOpMode {
    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor rlMotor;
    DcMotor rrMotor;

    CRServo intakeLeft;
    CRServo intakeRight;

    public void runOpMode() {
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rlMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double fbDrive = gamepad1.left_stick_y;
            double lrDrive = gamepad1.left_stick_x;
            double trDrive = gamepad1.right_stick_x;

            double flPower = fbDrive + lrDrive + trDrive;
            double frPower = fbDrive + lrDrive - trDrive;
            double rlPower = fbDrive - lrDrive + trDrive;
            double rrPower = fbDrive - lrDrive - trDrive;

            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            rlMotor.setPower(rlPower);
            rrMotor.setPower(rrPower);

            double intakeSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
            intakeLeft.setPower(intakeSpeed);
            intakeRight.setPower(intakeSpeed);
        }
    }
}
