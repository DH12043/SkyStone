package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "AutoREDPrimary", group= "None")
public class AutoREDPrimary extends SkystoneVuforiaNew {

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;

    @Override
    public void init() {
        super.init();
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
//        IntakeMotor = hardwareMap.dcMotor.get("Intake");
//        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        lFoundationator = hardwareMap.servo.get("Left Foundationator");
        rFoundationator = hardwareMap.servo.get("Right Foundationator");
    }

    @Override
    public void start() {

        LocateSkystone();

        if (positionSkystone == "Left") {
          DriveToSkystoneLeft();
        }
        else if (positionSkystone == "Center") {
          DriveToSkystoneCenter();
        }
        else if (positionSkystone == "Right") {
          DriveToSkystoneRight();
        }

        DriveToFoundation();            // DH's Thing

        MoveFoundation();

        Park();
    }

    private void LocateSkystone() {     // DH's Thing

    }

    private void DriveToSkystoneLeft() {

    }

    private void DriveToSkystoneCenter() {

    }

    private void DriveToSkystoneRight() {

    }

    private void DriveToFoundation() {  // DH's Thing

    }

    private void MoveFoundation() {
        double startTime = getRuntime();
        while (startTime <= (getRuntime() + 3)) {
            FrontRight.setPower(-1);
            FrontLeft.setPower(-1);
            BackRight.setPower(-1);
            BackLeft.setPower(-1);
        }
        while (startTime <= (getRuntime() - 10)) {
            FrontRight.setPower(-.7);
            FrontLeft.setPower(.7);
            BackRight.setPower(1);
            BackLeft.setPower(-1);
        }
    }

    private void Park() {
        double startTime = getRuntime();
        while (startTime <= (getRuntime() - 3)) {
            FrontRight.setPower(-1);
            FrontLeft.setPower(-1);
            BackRight.setPower(-1);
            BackLeft.setPower(-1);
        }
    }
}