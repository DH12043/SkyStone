package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class DJIntakeAutonomous extends LinearOpMode {
    private Servo IntakeReleaseServo;

    private double intakeNotReleased = 0;
    private double intakeReleased = .6;

    @Override
    public void runOpMode() {

        IntakeReleaseServo = hardwareMap.get(Servo.class, "IntakeReleaseServo");
        IntakeReleaseServo.setPosition(intakeNotReleased);

        waitForStart();

        setIntakeReleaseServo();

    }

    private void setIntakeReleaseServo () {
        IntakeReleaseServo.setPosition(intakeReleased);
    }
}
