package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "AutoREDPrimary", group= "None")
public class AutoREDPrimary extends SkystoneVuforiaNew {


    DcMotor verticalRight, verticalLeft, horizontal;

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


    final double COUNTS_PER_INCH = 307.699557;                  //TODO CHANGE
    private static final int FoundationXPosition = 48;
    private static final int FoundationYPosition = 107;
    private static final int BuildSiteXPosition = 9;
    private static final int BuildSiteYPosition = 111;
    private static final int ParkLineXPosition = 9;
    private static final int ParkLineYPosition = 72;

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
        verticalLeft = hardwareMap.dcMotor.get("verticalLeftEncoderName");
        verticalRight = hardwareMap.dcMotor.get("verticalRightEncoderName");
        horizontal = hardwareMap.dcMotor.get("horizontalEncoderName");
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotXPosition = (9);
        RobotYPosition = (36);
        RobotRotation = (0);
    }

    @Override
    public void start() {


        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

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

        globalPositionUpdate.stop();
    }

    private void DriveToSkystoneLeft() {
        while((RobotXPosition < SkystoneXPosition) || (RobotYPosition < SkystoneYPosition)) {
            FrontRight.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackLeft.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
        RobotXPosition = SkystoneXPosition;
        RobotYPosition = SkystoneYPosition;
    }

    private void DriveToSkystoneCenter() {
        while(RobotXPosition < SkystoneXPosition) {
            FrontRight.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackLeft.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
        RobotXPosition = SkystoneXPosition;
        RobotYPosition = SkystoneYPosition;
    }

    private void DriveToSkystoneRight() {
        while((RobotXPosition < SkystoneXPosition) && (RobotYPosition > SkystoneYPosition)) {
            FrontRight.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackRight.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackLeft.setPower(Math.sqrt(((SkystoneXPosition - RobotXPosition) * (SkystoneXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
        RobotXPosition = SkystoneXPosition;
        RobotYPosition = SkystoneYPosition;
    }

    private void DriveToFoundation() {
        while((RobotXPosition < FoundationXPosition) && (RobotYPosition > FoundationYPosition)) {
            FrontRight.setPower(Math.sqrt(-((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackRight.setPower(Math.sqrt(((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) - ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
            BackLeft.setPower(Math.sqrt(((FoundationXPosition - RobotXPosition) * (FoundationXPosition - RobotXPosition)) + ((SkystoneYPosition - RobotYPosition) * (SkystoneYPosition - RobotYPosition))));
        }
        RobotXPosition = FoundationXPosition;
        RobotYPosition = FoundationYPosition;
        while (RobotRotation < 90) {
            FrontRight.setPower(.5);
            FrontLeft.setPower(-.5);
            BackRight.setPower(.5);
            BackLeft.setPower(-.5);
        }
    }

    private void MoveFoundation() {
        while((RobotXPosition < BuildSiteXPosition) && (RobotYPosition > BuildSiteYPosition)) {
            FrontRight.setPower(Math.sqrt(-((BuildSiteXPosition - RobotXPosition) * (BuildSiteXPosition - RobotXPosition)) - ((BuildSiteYPosition - RobotYPosition) * (BuildSiteYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((BuildSiteXPosition - RobotXPosition) * (BuildSiteXPosition - RobotXPosition)) + ((BuildSiteYPosition - RobotYPosition) * (BuildSiteYPosition - RobotYPosition))));
            BackRight.setPower(Math.sqrt(((BuildSiteXPosition - RobotXPosition) * (BuildSiteXPosition - RobotXPosition)) - ((BuildSiteYPosition - RobotYPosition) * (BuildSiteYPosition - RobotYPosition))));
            BackLeft.setPower(Math.sqrt(((BuildSiteXPosition - RobotXPosition) * (BuildSiteXPosition - RobotXPosition)) + ((BuildSiteYPosition - RobotYPosition) * (BuildSiteYPosition - RobotYPosition))));
        }
    }

    private void Park() {
        while ((RobotXPosition < ParkLineXPosition) && (RobotYPosition > ParkLineYPosition)) {
            FrontRight.setPower(Math.sqrt(-((ParkLineXPosition - RobotXPosition) * (ParkLineXPosition - RobotXPosition)) - ((ParkLineYPosition - RobotYPosition) * (ParkLineYPosition - RobotYPosition))));
            FrontLeft.setPower(Math.sqrt(-((ParkLineXPosition - RobotXPosition) * (ParkLineXPosition - RobotXPosition)) + ((ParkLineYPosition - RobotYPosition) * (ParkLineYPosition - RobotYPosition))));
            BackRight.setPower(Math.sqrt(((ParkLineXPosition - RobotXPosition) * (ParkLineXPosition - RobotXPosition)) - ((ParkLineYPosition - RobotYPosition) * (ParkLineYPosition - RobotYPosition))));
            BackLeft.setPower(Math.sqrt(((ParkLineXPosition - RobotXPosition) * (ParkLineXPosition - RobotXPosition)) + ((ParkLineYPosition - RobotYPosition) * (ParkLineYPosition - RobotYPosition))));
        }
    }
}