package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import static java.lang.Thread.sleep;

@Autonomous (name= "AutoREDPrimary", group= "None")
public class AutoREDPrimary extends SkystoneVuforiaNew {

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    private File startingXpositionFile = AppUtil.getInstance().getSettingsFile("startingXposition.txt");
    private File startingYpositionFile = AppUtil.getInstance().getSettingsFile("startingYposition.txt");
    private File startingθpositionFile = AppUtil.getInstance().getSettingsFile("startingθposition.txt");

    Thread positionThread;
    OdometryGlobalCoordinatePosition globalPositionUpdate;

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


    final double COUNTS_PER_INCH = 307.699557;
    private static int PreFoundationXPosition = 36;
    private static int PreFoundationYPosition = 95;
    private static int FoundationXPosition = 48;
    private static int FoundationYPosition = 107;
    private static int BuildSiteXPosition = 9;
    private static int BuildSiteYPosition = 111;
    private static int ParkLineXPosition = 9;
    private static int ParkLineYPosition = 72;

    private double CurrentRobotXPosition;
    private double CurrentRobotYPosition;
    private double CurrentRobotRotation;

    @Override
    public void init() {
        super.init();
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
//        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        RobotXPosition = (9);
        RobotYPosition = (36);
        RobotRotation = (0);
    }

    @Override
    public void start() {
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        CurrentRobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotXPosition;
        CurrentRobotYPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotYPosition;
        CurrentRobotRotation = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotRotation;

        if (positionSkystone.equals("Left")) {
            SkystoneXPosition = (38);
            SkystoneYPosition = (44);
//            driveToSkystonePosition(globalPositionUpdate);
            IntakeMotor.setPower(1);
        }
        else if (positionSkystone.equals("Center")) {
            SkystoneXPosition = (38);
            SkystoneYPosition = (36);
//            driveToSkystonePosition(globalPositionUpdate);
            IntakeMotor.setPower(1);
        }
        else if (positionSkystone.equals("Right")) {
            SkystoneXPosition = (38);
            SkystoneYPosition = (32);
//            driveToSkystonePosition(globalPositionUpdate);
            IntakeMotor.setPower(1);
        }
        else {
            telemetry.addData("Skystone Location Error", "");
            telemetry.update();
            SkystoneXPosition = (38);
            SkystoneYPosition = (32);
            try {
                sleep(1400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
//            driveToSkystonePosition(globalPositionUpdate);
            IntakeMotor.setPower(1);
        }

//        DriveToFoundation(globalPositionUpdate);
//
//        HookOntoFoundation(globalPositionUpdate);
//
//        MoveFoundation(globalPositionUpdate);
//
//        Park(globalPositionUpdate);

        globalPositionUpdate.stop(); //Maybe comment this out?
    }

    /*@Override
    public void stop() {
        OdometryGlobalCoordinatePosition position = null;
        ReadWriteFile.writeFile(startingXpositionFile, String.valueOf(position.returnXCoordinate()));
        ReadWriteFile.writeFile(startingYpositionFile, String.valueOf(position.returnYCoordinate()));
        ReadWriteFile.writeFile(startingθpositionFile, String.valueOf(position.returnOrientation()));
    }*/

    double getDistanceFromCoordinates(double x, double y, OdometryGlobalCoordinatePosition position) {
        double deltaX = x - position.returnXCoordinate();
        double deltaY = y - position.returnYCoordinate();

        telemetry.addData("deltaX",deltaX);
        telemetry.addData("deltaY",deltaY);
        telemetry.update();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    double[] calculateMotorSpeeds(double deltaX, double deltaY) {
        double[] speeds = new double[4];
        speeds[0] = -deltaY + deltaX;
        speeds[1] = deltaY - deltaX;
        speeds[2] = -deltaY - deltaX;
        speeds[3] = deltaY + deltaX;


        if (speeds[0] > 1) {
            speeds[0] = 1;
        }
        else if (speeds[0] < -1) {
            speeds[0] = -1;
        }
        if (speeds[1] > 1) {
            speeds[1] = 1;
        }
        else if (speeds[1] < -1) {
            speeds[1] = -1;
        }
        if (speeds[2] > 1) {
            speeds[2] = 1;
        }
        else if (speeds[2] < -1) {
            speeds[2] = -1;
        }
        if (speeds[3] > 1) {
            speeds[3] = 1;
        }
        else if (speeds[3] < -1) {
            speeds[3] = -1;
        }
//        double maxSpeed = 0;
//        for(double speed : speeds) {
//            maxSpeed = Math.max(maxSpeed, Math.abs(speed));
//        }
//
//        if(maxSpeed > 1.0) {
//            for(int i = 0; i < 4; ++i) {
//                speeds[i] = maxSpeed;
//            }
//        }
//
        return speeds;
    }

    private void driveToSkystonePosition(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(SkystoneXPosition, SkystoneYPosition, position);

        while (position.returnYCoordinate() < SkystoneYPosition) {
            FrontRight.setPower(1);
            FrontLeft.setPower(-1);
            BackRight.setPower(-1);
            BackLeft.setPower(1);
        }
        while (position.returnYCoordinate() > SkystoneYPosition) {
            FrontRight.setPower(-1);
            FrontLeft.setPower(1);
            BackRight.setPower(1);
            BackLeft.setPower(-1);
        }
        while(position.returnXCoordinate() > SkystoneXPosition + 10) {
            FrontRight.setPower(1);
            FrontLeft.setPower(1);
            BackRight.setPower(1);
            BackLeft.setPower(1);
        }
//        while(distanceAway > 18.5) {
//            double deltaX = SkystoneXPosition - position.returnXCoordinate();
//            double deltaY = SkystoneYPosition - position.returnYCoordinate();
//
//            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);
//
//            FrontRight.setPower(motorSpeeds[2]);
//            FrontLeft.setPower(motorSpeeds[1]);
//            BackRight.setPower(motorSpeeds[1]);
//            BackLeft.setPower(motorSpeeds[2]);
//
//            distanceAway = getDistanceFromCoordinates(SkystoneXPosition, SkystoneYPosition, position);
//        }
//        while(distanceAway <= 11) {
//            double deltaX = SkystoneXPosition - position.returnXCoordinate();
//            double deltaY = SkystoneYPosition - position.returnYCoordinate();
//
//            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);
//
//            FrontRight.setPower(motorSpeeds[2]);
//            FrontLeft.setPower(motorSpeeds[1]);
//            BackRight.setPower(motorSpeeds[1]);
//            BackLeft.setPower(motorSpeeds[2]);
//
//            distanceAway = getDistanceFromCoordinates(SkystoneXPosition, SkystoneYPosition, position);
//        }
    }

    private void DriveToFoundation(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(PreFoundationXPosition, PreFoundationYPosition, position);

        while (distanceAway > 9.5) {
            double deltaX = PreFoundationXPosition - position.returnXCoordinate();
            double deltaY = PreFoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(PreFoundationXPosition, PreFoundationYPosition, position);
        }
        while (distanceAway != 0) {
            double deltaX = PreFoundationXPosition - position.returnXCoordinate();
            double deltaY = PreFoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(PreFoundationXPosition, PreFoundationYPosition, position);
        }
        while (RobotRotation < 90) {
            FrontRight.setPower(.5);
            FrontLeft.setPower(-.5);
            BackRight.setPower(.5);
            BackLeft.setPower(-.5);
        }
    }
    private void HookOntoFoundation(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(FoundationXPosition, FoundationYPosition, position);

        while(distanceAway > 9.5) {
            double deltaX = FoundationXPosition - position.returnXCoordinate();
            double deltaY = FoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(FoundationXPosition, FoundationYPosition, position);
        }
        while(distanceAway != 0) {
            double deltaX = FoundationXPosition - position.returnXCoordinate();
            double deltaY = FoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(FoundationXPosition, FoundationYPosition, position);
        }

        lFoundationator.setPosition(.27);
        rFoundationator.setPosition(0);
    }

    private void MoveFoundation(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(BuildSiteXPosition, BuildSiteYPosition, position);

        while(distanceAway > 9.5) {
            double deltaX = BuildSiteXPosition - position.returnXCoordinate();
            double deltaY = BuildSiteYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(BuildSiteXPosition, BuildSiteYPosition, position);
        }
        while(distanceAway != 0) {
            double deltaX = BuildSiteXPosition - position.returnXCoordinate();
            double deltaY = BuildSiteYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(BuildSiteXPosition, BuildSiteYPosition, position);
        }

        lFoundationator.setPosition(0);
        rFoundationator.setPosition(.27);

        while (RobotRotation < 90) {
            FrontRight.setPower(.5);
            FrontLeft.setPower(-.5);
            BackRight.setPower(.5);
            BackLeft.setPower(-.5);
        }
    }

    private void Park(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(ParkLineXPosition, ParkLineYPosition, position);

        while(distanceAway > 9.5) {
            double deltaX = ParkLineXPosition - position.returnXCoordinate();
            double deltaY = ParkLineYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(ParkLineXPosition, ParkLineYPosition, position);
        }
        while(distanceAway != 0) {
            double deltaX = ParkLineXPosition - position.returnXCoordinate();
            double deltaY = ParkLineYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(ParkLineXPosition, ParkLineYPosition, position);
        }
    }
}