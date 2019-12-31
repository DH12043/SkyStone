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
    private double startTime;
    private double currentTime;

    private int autoState = 0;
    private int lastAutoState = -1;

    @Override
    public void init() {
        super.init();
        msStuckDetectStart = 300000;
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
//        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        RobotYPosition = (-36);
        RobotRotation = (90);
    }

    @Override
    public void start() {
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        CurrentRobotXPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + RobotXPosition;
        CurrentRobotYPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotYPosition;
        CurrentRobotRotation = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotRotation;

        telemetry.addData ("RobotX", CurrentRobotXPosition);
        telemetry.addData ("RobotY", CurrentRobotYPosition);
        telemetry.addData ("RobotRotation", CurrentRobotRotation);

        // Move to loop ---------------------------------------------------------------

    }

    @Override
    public void loop() {
        currentTime = getRuntime();
        // Drop intake
        // Turn on intake
        if (autoState == 0) {
                                                                    // TODO setPosition of the servo
            IntakeMotor.setPower(1);

            autoState++;
            lastAutoState = 0;
        }

        // Drive to stone / pick it up
        else if (autoState == 1) {
            // Will only happen upon entering state 1
            if (lastAutoState == 0) {
                startTime = getRuntime();
                if (positionSkystone.equals("Left")) {
                    telemetry.addData("Skystone Location Left", "");
                    SkystoneXPosition = (47);
                    SkystoneYPosition = (44);
                } else if (positionSkystone.equals("Center")) {
                    telemetry.addData("Skystone Location Center", "");
                    SkystoneXPosition = (47);
                    SkystoneYPosition = (36);
                } else if (positionSkystone.equals("Right")) {
                    telemetry.addData("Skystone Location Right", "");
                    SkystoneXPosition = (47);
                    SkystoneYPosition = (28);
                } else {
                    telemetry.addData("Skystone Location Error", "");
                    SkystoneXPosition = (47);
                    SkystoneYPosition = (28);
                }
            }
            if (startTime > currentTime - 5) {
                telemetry.addData("startTime", startTime);
                telemetry.addData("currentTime", currentTime);
                telemetry.addData("Time Left", currentTime - (startTime + 5));
                telemetry.update();
            }
//            driveToSkystonePosition(globalPositionUpdate);
            else {
                autoState++;
            }
            lastAutoState = 1;
        }
        else if(autoState == 2) {
            if(lastAutoState == 1) {
                IntakeMotor.setPower(0);
            }
            lastAutoState = 2;
        }
//        DriveToFoundation(globalPositionUpdate);
//
//        HookOntoFoundation(globalPositionUpdate);
//
//        MoveFoundation(globalPositionUpdate);
//
//        Park(globalPositionUpdate);

    }

    public void driveToScoringPosition() {
        java.lang.String turningDirection;
        double distanceFromOrientation;

        if (RobotRotation > 2 && RobotRotation < 180) {
            turningDirection = "left";
        }
        else {
            turningDirection = "right";
        }
        telemetry.addData("TurningDirection", turningDirection);
        if (RobotRotation > 2 && RobotRotation < 358) {
            if (turningDirection.equals("right")) {
                distanceFromOrientation = 360 - RobotRotation;
                if (distanceFromOrientation > 120) {
                    Drive(0, 0, 1);
                }
                else if (distanceFromOrientation > 45) {
                    Drive(0,0,.35);
                }
                else {
                    Drive(0,0,.15);
                }
            }
            else {
                distanceFromOrientation = RobotRotation;
                if (distanceFromOrientation > 120) {
                    Drive(0, 0, -1);
                }
                else if (distanceFromOrientation > 45) {
                    Drive(0,0,-.35);
                }
                else {
                    Drive(0,0,-.15);
                }
            }
            telemetry.addData("DistanceFromOrientation", distanceFromOrientation);
        }
        else if (RobotYPosition < -.5 || RobotYPosition > .5) {
            if (RobotYPosition < 0) {
                if (RobotYPosition < -6) {
                    Drive(-.3,0,0);
                }
                else {
                    Drive(-.1, 0, 0);
                }
            }
            else {
                if (RobotYPosition > 6) {
                    Drive(.3,0,0);
                }
                else {
                    Drive(.1,0,0);
                }
            }
        }
        else if (RobotXPosition < -.5 || RobotXPosition > .5) {
            if (RobotXPosition < 0) {
                if (RobotXPosition < -6) {
                    Drive(0,.5,0);
                }
                else {
                    Drive(0, .2, 0);
                }
            }
            else {
                if (RobotXPosition > 6) {
                    Drive(0,-.5,0);
                }
                else {
                    Drive(0,-.2,0);
                }
            }
        }
        else {
            Drive(0,0,0);
        }
    }

    public void Drive(double DZForwardButton, double DZSidewaysButton, double DZSpinningButton) {
        DZSpinningButton = -DZSpinningButton;
        FrontRight.setPower(-DZSidewaysButton - DZForwardButton + DZSpinningButton);
        FrontLeft.setPower(-DZSidewaysButton + DZForwardButton + DZSpinningButton);
        BackRight.setPower(DZSidewaysButton - DZForwardButton + DZSpinningButton);
        BackLeft.setPower(DZSidewaysButton + DZForwardButton + DZSpinningButton);
    }

    @Override
    public void stop() {
        globalPositionUpdate.stop(); //Maybe comment this out?
//        OdometryGlobalCoordinatePosition position = null;
//        ReadWriteFile.writeFile(startingXpositionFile, String.valueOf(position.returnXCoordinate()));
//        ReadWriteFile.writeFile(startingYpositionFile, String.valueOf(position.returnYCoordinate()));
//        ReadWriteFile.writeFile(startingθpositionFile, String.valueOf(position.returnOrientation()));
    }

    double getDistanceFromCoordinates(double x, double y, OdometryGlobalCoordinatePosition position) {
        double deltaX = x - position.returnXCoordinate();
        double deltaY = y - position.returnYCoordinate();

//        telemetry.addData("deltaX",deltaX);
//        telemetry.addData("deltaY",deltaY);
//        telemetry.update();

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

        if(CurrentRobotXPosition < SkystoneXPosition) {
            if(CurrentRobotYPosition < SkystoneYPosition) {
                FrontRight.setPower(.3);
                FrontLeft.setPower(.3);
                BackRight.setPower(-.3);
                BackLeft.setPower(-.3);
                telemetry.addData("Strafing","Forward Right");
                telemetry.update();
            }
            else if(CurrentRobotYPosition > SkystoneYPosition) {
                FrontRight.setPower(-.3);
                FrontLeft.setPower(-.3);
                BackRight.setPower(.3);
                BackLeft.setPower(.3);
                telemetry.addData("Strafing", "Forward Left");
            }
            else {
                FrontRight.setPower(.15);
                FrontLeft.setPower(-.15);
                BackRight.setPower(.15);
                BackLeft.setPower(-.15);
                telemetry.addData("Straight", "Forward");
                telemetry.addData("RobotX",CurrentRobotXPosition);
                telemetry.addData("RobotY",CurrentRobotYPosition);
                telemetry.addData("RobotRotation", CurrentRobotRotation);
            }
            CurrentRobotXPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + RobotXPosition;
            CurrentRobotYPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotYPosition;
            CurrentRobotRotation = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotRotation;
            telemetry.addData("SkystoneXPosition", SkystoneXPosition / COUNTS_PER_INCH);
            telemetry.addData("SkystoneYPosition", SkystoneYPosition / COUNTS_PER_INCH);
            telemetry.update();
        }
        while (position.returnXCoordinate() > SkystoneXPosition - 20) {
            FrontRight.setPower(-15);
            FrontLeft.setPower(.15);
            BackRight.setPower(-.15);
            BackLeft.setPower(.15);
        }
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);
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