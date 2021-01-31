package org.firstinspires.ftc.teamcode.libraries;

public class AutonomousFunctions {
    private Alexi robot;
    private TeleAuto callback;

    public AutonomousFunctions(Alexi alexi, TeleAuto callback) {
        this.robot = alexi;
        this.callback = callback;
    }

    public void shooterOn() {
        this.robot.wobbleMotor.setPower(1);
    }

    public void shooterOff() {
        this.robot.wobbleMotor.setPower(0);
    }

    public void shoot() {
        this.robot.wobbleServo.setPosition(-1);
        callback.sleep(1000);
        this.robot.wobbleServo.setPosition(1);
    }
}
