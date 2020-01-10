package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    private Servo IntakeReleaseServo;

    final double COUNTS_PER_INCH = 307.699557;
    private double CurrentRobotXPosition;
    private double CurrentRobotYPosition;
    private double CurrentRobotRotation;
    private double startTime;
    private double currentTime;
    private double intakeNotReleased = .6;
    private double intakeReleased = .15;
    private double autoState = 0;
    private double lastAutoState = -1;
    private double lastDistanceToTarget = 0;
    private double RobotXPosition = 9;
    private double RobotYPosition = 36;
    private double RobotRotation = 90;
    private double movement_x;
    private double movement_y;
    private double movement_turn;
    private boolean autoComplete = true;
    private static final double DECELERATION_START_POINT = 96;
    private static final double DECELERATION_ZERO_POINT = -6;
    private static final double TURNING_DECELERATION_START_POINT = 180;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5;
    private static final double X_SPEED_MULTIPLIER = 1;
    private long lastUpdateTime = 0;

    @Override
    public void init() {
        super.init();
        msStuckDetectStart = 300000;
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        IntakeReleaseServo = hardwareMap.get(Servo.class, "IntakeReleaseServo");
        IntakeReleaseServo.setPosition(intakeNotReleased);
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
        RobotXPosition = 9;
        RobotYPosition = 36;
        RobotRotation = 90;
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
        telemetry.addData("autoState", autoState);
        telemetry.addData("lastAutoState", lastAutoState);
        telemetry.addData("CurrentRobotYPosition", CurrentRobotYPosition);
        telemetry.addData("CurrentRobotXPosition", CurrentRobotXPosition);
        telemetry.addData("startTime", startTime);
        telemetry.addData("currentTime", currentTime);
        telemetry.addData("Time Left", currentTime - (startTime + 5));
        if(autoState == 0) {
            if (lastAutoState == -1) {
                startTime = getRuntime();
                lastAutoState = 0;
            } else if (startTime > currentTime - 5 && autoComplete == false) {
                if (CurrentRobotYPosition < 72) {
                    driveToPark(globalPositionUpdate);
                } else {
                    autoComplete = false;
                }
            } else {
                autoState++;
            }
        }
    }

    @Override
    public void stop() {
        globalPositionUpdate.stop();
    }

    public void driveToPark(OdometryGlobalCoordinatePosition position) {
        goToPositionMrK(9,72,.3, .3,90);
    }


    /**
     * Universal goToPosition method that assumes the following coordinate system for the
     * odometry output values: , RobotYPosition, and :
     * RobotRotation: 0*->360* rotation values, where 0* and 360* are straight forward
     * RobotXPosition: Positive X would be strafing to the right
     * RobotYPosition: Positive Y would be moving forwards
     * Method generates three values indicating the immediate motion needed to move the
     * robot towards the global position requested:
     * movement_turn: Positive value to turn clockwise, negative for counterclockwise
     * movement_x: Positive value indicates strafe to the right proportionally
     * movement_y: Positive value indicates drive forward proportionally
     * These values can be used as inputs to a method that typically accepts joystick control,
     * in some cases the 'y' value must be negated.
     *
     * author: Original code by FTC Team 11115 Gluten Free, modified by Travis Kuiper.
     * date: 2020/01/01
     *
     * @param x global target coordinate 'x' component
     * @param y global target coordinate 'y' component
     * @param maxMovementSpeed max speed value to be given to any one drivetrain direction of motion
     * @param maxTurnSpeed max turning speed to be given to drivetrain rotation
     * @param preferredAngle global target coordinate theta component
     */
    private void goToPositionMrK(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle) {
        double distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);
        double relativeAngleToPoint = AngleWrap(-absoluteAngleToTarget
                - Math.toRadians(RobotRotation) + Math.toRadians(90));

        double relativeXToPoint = 2 * Math.sin(relativeAngleToPoint);
        double relativeYToPoint = Math.cos(relativeAngleToPoint);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double yDecelLimiter = Range.clip(Math.abs((distanceToTarget - DECELERATION_ZERO_POINT)
                / (DECELERATION_START_POINT - DECELERATION_ZERO_POINT)), 0, 1);
        double xDecelLimiter = Range.clip(yDecelLimiter * X_SPEED_MULTIPLIER, 0, 1);

        double relativeTurnAngle = AngleWrap(Math.toRadians(preferredAngle)-Math.toRadians(RobotRotation));
        double turnDecelLimiter = Range.clip((Math.abs(Math.toDegrees(relativeTurnAngle)) - TURNING_DECELERATION_ZERO_POINT)
                / (TURNING_DECELERATION_START_POINT - TURNING_DECELERATION_ZERO_POINT), 0, 1);

        movement_x = movementXPower * Range.clip(maxMovementSpeed, -xDecelLimiter, xDecelLimiter);
        movement_y = movementYPower * Range.clip(maxMovementSpeed, -yDecelLimiter, yDecelLimiter);

        if (distanceToTarget < 1) {
            movement_turn = 0;
        } else {
            //movement_turn = Range.clip(Range.clip(relativeTurnAngle / Math.toRadians(30),
            //        -1, 1) * maxTurnSpeed, -turnDecelLimiter, turnDecelLimiter);
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(TURNING_DECELERATION_START_POINT), -1, 1) * maxTurnSpeed;
        }
        telemetry.addData("relativeTurnAngle", relativeTurnAngle);
        telemetry.addData("turnDecelLimiter", turnDecelLimiter);
        telemetry.addData("relativeXToPoint", relativeXToPoint);
        telemetry.addData("relativeYToPoint", relativeYToPoint);
        telemetry.addData("X Movement", movement_x);
        telemetry.addData("Y Movement", movement_y);
        telemetry.addData("Turn Movement", movement_turn);

        lastDistanceToTarget = distanceToTarget;

        applyMovement();
    }

    /**
     * Converts movement_y, movement_x, movement_turn into motor powers.
     *
     * author: Original code by FTC Team 7571 Alumineers, modified by Travis Kuiper
     * date: 2020/01/01
     *
     * movement_turn: Positive value to turn clockwise, negative for counterclockwise
     * movement_x: Positive value indicates strafe to the right proportionally
     * movement_y: Positive value indicates drive forward proportionally
     */
    // Code comes from 11115 Peter and 7571 Alumineers
    private void applyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        //double fl_power_raw = movement_y-movement_turn+movement_x;
        //double bl_power_raw = movement_y-movement_turn-movement_x;
        //double br_power_raw = -movement_y-movement_turn-movement_x;
        //double fr_power_raw = -movement_y-movement_turn+movement_x;

        double fl_power_raw = movement_y+movement_turn+movement_x;
        double bl_power_raw = movement_y+movement_turn-movement_x;
        double br_power_raw = -movement_y+movement_turn-movement_x;
        double fr_power_raw = -movement_y+movement_turn+movement_x;

        //find the maximum of the powers
        double maxRawPower = Math.abs(fl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(fr_power_raw) > maxRawPower){ maxRawPower = Math.abs(fr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        fl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        fr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        FrontLeft.setPower(-fl_power_raw);
        BackLeft.setPower(-bl_power_raw);
        BackRight.setPower(-br_power_raw);
        FrontRight.setPower(-fr_power_raw);
    }
    private static double AngleWrap(double angle){

        while(angle < -Math.PI){
            angle += 2*Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2*Math.PI;
        }

        return angle;
    }
}
