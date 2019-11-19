package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "AutoREDPrimary", group= "None")
public class AutoREDPrimary extends LinearOpMode {

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;

    @Override
    public void runOpMode() {

        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        IntakeMotor = hardwareMap.dcMotor.get("Intake");
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        lFoundationator = hardwareMap.servo.get("Left Foundationator");
        rFoundationator = hardwareMap.servo.get("Right Foundationator");


        waitForStart();

        LocateSkystone();               // DH's Thing

        DriveToSkystone();

        DriveToFoundation();            // DH's Thing

        MoveFoundation();

        Park();
    }

    private void LocateSkystone() {     // DH's Thing

    }

    private void DriveToSkystone(/*double RobotXCoordinate, double RobotYCoordinate, double TargetXCoordinate, double TargetYCoordinate*/) {
/*        FrontRight.setPower((-TargetXCoordinate - TargetYCoordinate) + (RobotXCoordinate - RobotYCoordinate));
        FrontLeft.setPower((-TargetXCoordinate + TargetYCoordinate) + (RobotXCoordinate - RobotYCoordinate));
        BackRight.setPower((TargetXCoordinate - TargetYCoordinate) + (RobotXCoordinate - RobotYCoordinate));
        BackLeft.setPower((TargetXCoordinate + TargetYCoordinate) + (RobotXCoordinate - RobotYCoordinate));
*/    }

    private void DriveToFoundation() {  // DH's Thing

    }

    private void MoveFoundation(/*double MovingTime*/) {
/*        while (MovingTime <= 10) {
            FrontRight.setPower(-1);
            FrontLeft.setPower(-1);
            BackRight.setPower(-1);
            BackLeft.setPower(-1);

            MovingTime = MovingTime + .5;
        }
        while ((MovingTime <= 40) && (MovingTime > 10)) {
            FrontRight.setPower(-.7);
            FrontLeft.setPower(.7);
            BackRight.setPower(1);
            BackLeft.setPower(-1);

            MovingTime = MovingTime + .5;
        }
*/    }

    private void Park() {

        FrontRight.setPower(1);

    }
}