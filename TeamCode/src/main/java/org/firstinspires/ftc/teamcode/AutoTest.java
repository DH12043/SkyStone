package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "AutoTest", group= "None")
public class AutoTest extends SkystoneVuforiaNew {

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    Thread positionThread;
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;

    final double COUNTS_PER_INCH = 307.699557;
    private double CurrentRobotXPosition;
    private double CurrentRobotYPosition;
    private double CurrentRobotRotation;
    private double startTime;
    private double currentTime;

    @Override
    public void init() {
        super.init();
        msStuckDetectStart = 300000;
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        RobotRotation = (90);

    }

    @Override
    public void start() {

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        CurrentRobotXPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + RobotXPosition;
        CurrentRobotYPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotYPosition;
        CurrentRobotRotation = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + RobotRotation;
    }

    @Override
    public void loop() {
        currentTime = getRuntime();
        while (startTime > currentTime - 5) {
            driveToPark(globalPositionUpdate);
        }
    }

    @Override
    public void stop() {
        globalPositionUpdate.stop();
    }

    public void driveToPark(OdometryGlobalCoordinatePosition position) {
        DriveTo(9, 72);
    }

    public void DriveTo(double x, double y) {
        double FRSpeed = (-x - y);
        double FLSpeed = (-x + y);
        double BRSpeed = (x - y);
        double BLSpeed = (x + y);


        if(FRSpeed > FLSpeed) {
            if (FRSpeed > BRSpeed) {
                if (FRSpeed > BLSpeed) {
                    FRSpeed = (FRSpeed / FRSpeed);
                    FLSpeed = (FLSpeed / FRSpeed);
                    BRSpeed = (BRSpeed / FRSpeed);
                    BLSpeed = (BLSpeed / FRSpeed);
                }

                else if (FRSpeed < BLSpeed) {
                    FRSpeed = (FRSpeed / BLSpeed);
                    FLSpeed = (FLSpeed / BLSpeed);
                    BRSpeed = (BRSpeed / BLSpeed);
                    BLSpeed = (BLSpeed / BLSpeed);
                }
            }


            else if (FRSpeed < BRSpeed) {
                if (BRSpeed > BLSpeed) {
                    FRSpeed = (FRSpeed / BRSpeed);
                    FLSpeed = (FLSpeed / BRSpeed);
                    BRSpeed = (BRSpeed / BRSpeed);
                    BLSpeed = (BLSpeed / BRSpeed);
                }

                else if (FRSpeed < BLSpeed) {
                    FRSpeed = (FRSpeed / BLSpeed);
                    FLSpeed = (FLSpeed / BLSpeed);
                    BRSpeed = (BRSpeed / BLSpeed);
                    BLSpeed = (BLSpeed / BLSpeed);
                }
            }
        }



        else if(FRSpeed < FLSpeed) {
            if(FLSpeed > BRSpeed) {
                if(FLSpeed > BLSpeed) {
                    FRSpeed = (FRSpeed / FLSpeed);
                    FLSpeed = (FLSpeed / FLSpeed);
                    BRSpeed = (BRSpeed / FLSpeed);
                    BLSpeed = (BLSpeed / FLSpeed);
                }

                else if (FLSpeed < BLSpeed) {
                    FRSpeed = (FRSpeed / BLSpeed);
                    FLSpeed = (FLSpeed / BLSpeed);
                    BRSpeed = (BRSpeed / BLSpeed);
                    BLSpeed = (BLSpeed / BLSpeed);
                }
            }


            else if(FLSpeed < BRSpeed) {
                if(BRSpeed > BLSpeed) {
                    FRSpeed = (FRSpeed / BRSpeed);
                    FLSpeed = (FLSpeed / BRSpeed);
                    BRSpeed = (BRSpeed / BRSpeed);
                    BLSpeed = (BLSpeed / BRSpeed);
                }

                else if (BRSpeed < BLSpeed) {
                    FRSpeed = (FRSpeed / BLSpeed);
                    FLSpeed = (FLSpeed / BLSpeed);
                    BRSpeed = (BRSpeed / BLSpeed);
                    BLSpeed = (BLSpeed / BLSpeed);
                }
            }
        }

        while (CurrentRobotYPosition < y) {
            FrontRight.setPower(FRSpeed);
            FrontLeft.setPower(FLSpeed);
            BackRight.setPower(BRSpeed);
            BackLeft.setPower(BLSpeed);
        }
    }
}