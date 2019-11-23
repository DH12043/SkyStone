package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "AutoREDPrimary", group= "None")
public class AutoREDPrimary extends SkystoneVuforiaNew {

    private int SkystoneXPosition;
    private int SkystoneYPosition;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;

    private static final int FoundationXPosition = 48;
    private static final int FoundationYPosition = 107;

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
        RobotXPosition = (9);
        RobotYPosition = (36);
    }

    @Override
    public void start() {

        if (positionSkystone == "Left") {
            SkystoneXPosition = (38);
            SkystoneYPosition = (44);
            DriveToSkystoneLeft();
        }
        else if (positionSkystone == "Center") {
            SkystoneXPosition = (38);
            SkystoneYPosition = (36);
            DriveToSkystoneCenter();
        }
        else if (positionSkystone == "Right") {
            SkystoneXPosition = (38);
            SkystoneYPosition = (32);
            DriveToSkystoneRight();
        }
        else {
            telemetry.addData("Skystone Location Error", "");
            telemetry.update();
        }

        DriveToFoundation();            // DH's Thing

        MoveFoundation();

        Park();
    }

    private void DriveToSkystoneLeft() {
        while((RobotXPosition < SkystoneXPosition) || (RobotYPosition < SkystoneYPosition)) {
            FrontRight.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
    }

    private void DriveToSkystoneCenter() {
        while(RobotXPosition < SkystoneXPosition) {
            FrontRight.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
    }

    private void DriveToSkystoneRight() {
        while((RobotXPosition < SkystoneXPosition) && (RobotYPosition > SkystoneYPosition)) {
            FrontRight.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
    }

    private void DriveToFoundation() {
        while((RobotXPosition < FoundationXPosition) && (RobotYPosition > FoundationYPosition)) {
            FrontRight.setPower(Math.sqrt(-((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontRight.setPower(Math.sqrt(((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
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