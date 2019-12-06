package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "AutoREDSecondary", group= "None")
public class AutoREDSecondary extends SkystoneVuforiaNew {


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
    private static final int PreSkystoneXPosition = 9;
    private static final int PreSkystoneYPosition = 36;
    private static final int PreFoundationXPosition = 36;
    private static final int PreFoundationYPosition = 95;
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
        RobotYPosition = (96);
        RobotRotation = (0);
    }

    @Override
    public void start() {


        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        DriveToFoundation(globalPositionUpdate);

        MoveFoundation(globalPositionUpdate);

        PickUpSkystone(globalPositionUpdate);

        if (positionSkystone == "Left") {
            SkystoneXPosition = (38);
            SkystoneYPosition = (44);
            driveToSkystonePosition(globalPositionUpdate);
        }
        else if (positionSkystone == "Center") {
            SkystoneXPosition = (38);
            SkystoneYPosition = (36);
            driveToSkystonePosition(globalPositionUpdate);
        }
        else if (positionSkystone == "Right") {
            SkystoneXPosition = (38);
            SkystoneYPosition = (32);
            driveToSkystonePosition(globalPositionUpdate);
        }
        else {
            telemetry.addData("Skystone Location Error", "");
            telemetry.update();
        }

        Park(globalPositionUpdate);

        globalPositionUpdate.stop();
    }

    private void driveToSkystonePosition(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(SkystoneXPosition, SkystoneYPosition, position);

        while(distanceAway > 9.5) {
            double deltaX = SkystoneXPosition - position.returnXCoordinate();
            double deltaY = SkystoneYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[3]);

            distanceAway = getDistanceFromCoordinates(SkystoneXPosition, SkystoneYPosition, position);
        }
    }

    double getDistanceFromCoordinates(double x, double y, OdometryGlobalCoordinatePosition position) {
        double deltaX = x - position.returnXCoordinate();
        double deltaY = y - position.returnYCoordinate();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    double[] calculateMotorSpeeds(double deltaX, double deltaY) {
        double[] speeds = new double[4];
        speeds[0] = deltaY + deltaX;    // front-right
        speeds[1] = deltaY - deltaX;    // front-left
        speeds[2] = deltaY - deltaX;    // back-right
        speeds[3] = deltaY + deltaX;    // back-left

        double maxSpeed = 0;
        for(double speed : speeds) {
            maxSpeed = Math.max(maxSpeed, Math.abs(speed));
        }

        if(maxSpeed > 1.0) {
            for(int i = 0; i < 4; ++i) {
                speeds[i] /= maxSpeed;
            }
        }

        return speeds;
    }

    private void DriveToFoundation(OdometryGlobalCoordinatePosition position) {
        while((position.returnXCoordinate() < PreFoundationXPosition) && (position.returnYCoordinate() > PreFoundationYPosition)) {
            FrontRight.setPower(Math.sqrt(-((PreFoundationXPosition - position.returnXCoordinate()) * (PreFoundationXPosition - position.returnXCoordinate())) - ((PreFoundationYPosition - position.returnYCoordinate()) * (PreFoundationYPosition - position.returnYCoordinate()))));
            FrontLeft.setPower(Math.sqrt(-((PreFoundationXPosition - position.returnXCoordinate()) * (PreFoundationXPosition - position.returnXCoordinate())) + ((PreFoundationYPosition - position.returnYCoordinate()) * (PreFoundationYPosition - position.returnYCoordinate()))));
            BackRight.setPower(Math.sqrt(((PreFoundationXPosition - position.returnXCoordinate()) * (PreFoundationXPosition - position.returnXCoordinate())) - ((PreFoundationYPosition - position.returnYCoordinate()) * (PreFoundationYPosition - position.returnYCoordinate()))));
            BackLeft.setPower(Math.sqrt(((PreFoundationXPosition - position.returnXCoordinate()) * (PreFoundationXPosition - position.returnXCoordinate())) + ((PreFoundationYPosition - position.returnYCoordinate()) * (PreFoundationYPosition - position.returnYCoordinate()))));
        }
        while (RobotRotation < 90) {
            FrontRight.setPower(.5);
            FrontLeft.setPower(-.5);
            BackRight.setPower(.5);
            BackLeft.setPower(-.5);
        }
        while((position.returnXCoordinate() < FoundationXPosition) && (position.returnYCoordinate() > FoundationYPosition)) {
            FrontRight.setPower(Math.sqrt(-((FoundationXPosition - position.returnXCoordinate()) * (FoundationXPosition - position.returnXCoordinate())) - ((FoundationYPosition - position.returnYCoordinate()) * (FoundationYPosition - position.returnYCoordinate()))));
            FrontLeft.setPower(Math.sqrt(-((FoundationXPosition - position.returnXCoordinate()) * (FoundationXPosition - position.returnXCoordinate())) + ((FoundationYPosition - position.returnYCoordinate()) * (FoundationYPosition - position.returnYCoordinate()))));
            BackRight.setPower(Math.sqrt(((FoundationXPosition - position.returnXCoordinate()) * (FoundationXPosition - position.returnXCoordinate())) - ((FoundationYPosition - position.returnYCoordinate()) * (FoundationYPosition - position.returnYCoordinate()))));
            BackLeft.setPower(Math.sqrt(((FoundationXPosition - position.returnXCoordinate()) * (FoundationXPosition - position.returnXCoordinate())) + ((FoundationYPosition - position.returnYCoordinate()) * (FoundationYPosition - position.returnYCoordinate()))));
        }

        lFoundationator.setPosition(.27);
        rFoundationator.setPosition(0);
    }

    private void MoveFoundation(OdometryGlobalCoordinatePosition position) {
        while((position.returnXCoordinate() < BuildSiteXPosition) && (position.returnYCoordinate() > BuildSiteYPosition)) {
            FrontRight.setPower(Math.sqrt(-((BuildSiteXPosition - position.returnXCoordinate()) * (BuildSiteXPosition - position.returnXCoordinate())) - ((BuildSiteYPosition - position.returnYCoordinate()) * (BuildSiteYPosition - position.returnYCoordinate()))));
            FrontLeft.setPower(Math.sqrt(-((BuildSiteXPosition - position.returnXCoordinate()) * (BuildSiteXPosition - position.returnXCoordinate())) + ((BuildSiteYPosition - position.returnYCoordinate()) * (BuildSiteYPosition - position.returnYCoordinate()))));
            BackRight.setPower(Math.sqrt(((BuildSiteXPosition - position.returnXCoordinate()) * (BuildSiteXPosition - position.returnXCoordinate())) - ((BuildSiteYPosition - position.returnYCoordinate()) * (BuildSiteYPosition - position.returnYCoordinate()))));
            BackLeft.setPower(Math.sqrt(((BuildSiteXPosition - position.returnXCoordinate()) * (BuildSiteXPosition - position.returnXCoordinate())) + ((BuildSiteYPosition - position.returnYCoordinate()) * (BuildSiteYPosition - position.returnYCoordinate()))));
        }

        lFoundationator.setPosition(0);
        rFoundationator.setPosition(.27);
    }

    private void PickUpSkystone(OdometryGlobalCoordinatePosition position) {
        while(position.returnYCoordinate() < PreSkystoneYPosition) {
            FrontRight.setPower(Math.sqrt(-((PreSkystoneYPosition - position.returnYCoordinate()) * (PreSkystoneYPosition - position.returnYCoordinate()))));
            FrontLeft.setPower(Math.sqrt(((PreSkystoneYPosition - position.returnYCoordinate()) * (PreSkystoneYPosition - position.returnYCoordinate()))));
            BackRight.setPower(Math.sqrt(-((PreSkystoneYPosition - position.returnYCoordinate()) * (PreSkystoneYPosition - position.returnYCoordinate()))));
            BackLeft.setPower(Math.sqrt(((PreSkystoneYPosition - position.returnYCoordinate()) * (PreSkystoneYPosition - position.returnYCoordinate()))));
        }
    }

    private void Park(OdometryGlobalCoordinatePosition position) {
        while ((position.returnXCoordinate() < ParkLineXPosition) && (position.returnYCoordinate() > ParkLineYPosition)) {
            FrontRight.setPower(Math.sqrt(-((ParkLineXPosition - position.returnXCoordinate()) * (ParkLineXPosition - position.returnXCoordinate())) - ((ParkLineYPosition - position.returnYCoordinate()) * (ParkLineYPosition - position.returnYCoordinate()))));
            FrontLeft.setPower(Math.sqrt(-((ParkLineXPosition - position.returnXCoordinate()) * (ParkLineXPosition - position.returnXCoordinate())) + ((ParkLineYPosition - position.returnYCoordinate()) * (ParkLineYPosition - position.returnYCoordinate()))));
            BackRight.setPower(Math.sqrt(((ParkLineXPosition - position.returnXCoordinate()) * (ParkLineXPosition - position.returnXCoordinate())) - ((ParkLineYPosition - position.returnYCoordinate()) * (ParkLineYPosition - position.returnYCoordinate()))));
            BackLeft.setPower(Math.sqrt(((ParkLineXPosition - position.returnXCoordinate()) * (ParkLineXPosition - position.returnXCoordinate())) + ((ParkLineYPosition - position.returnYCoordinate()) * (ParkLineYPosition - position.returnYCoordinate()))));
        }
    }
}
