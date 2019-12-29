package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;


@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    public static final double DEADZONE = 0.15;

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;

    private double StartingXPosition;
    private double StartingYPosition;
    private double StartingRotation;

    private File startingXpositionFile = AppUtil.getInstance().getSettingsFile("startingXposition.txt");
    private File startingYpositionFile = AppUtil.getInstance().getSettingsFile("startingYposition.txt");
    private File startingθpositionFile = AppUtil.getInstance().getSettingsFile("startingθposition.txt");

    final double COUNTS_PER_INCH = 307.699557;
    private static final int PreFoundationXPosition = 36;
    private static final int PreFoundationYPosition = 95;
    private static final int FoundationXPosition = 48;
    private static final int FoundationYPosition = 107;
    private static final int BuildSiteXPosition = 9;
    private static final int BuildSiteYPosition = 111;
    private static final int ParkLineXPosition = 9;
    private static final int ParkLineYPosition = 72;

    private static final double GRABBERSERVOCLOSEDPOSITION = 0;
    private static final double GRABBERSERVOOPENPOSITION = .5;

    private int autoDrivingStage = 0;
    private int autoDrivingTimes = 0;

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private double rightBackupDistance;
    private double leftBackupDistance;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;
    private Servo GrabberServo;
    private Servo OrientStoneServo;
    private Servo MoveArmServo;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;
    private CRServo IntakeAssistServo;
    private Servo IntakeReleaseServo;

    private ModernRoboticsI2cRangeSensor OrientationSensor;
    private ModernRoboticsI2cRangeSensor StonePresenceSensor;

    private ModernRoboticsI2cRangeSensor RightBackupSensor;
    private ModernRoboticsI2cRangeSensor LeftBackupSensor;

    private int intakeState = 0;
    private int intakeReleaseState = 1;
    private int liftHeight = 0;
    private boolean firstLiftUpButton;
    private boolean firstLiftDownButton;
    private boolean liftUpCommand;
    private boolean liftDownCommand;
    private boolean outputButton;
    private boolean intakeButton;
    private boolean firstPressDpadUp = true;
    private boolean firstPressBumpers = true;
    private boolean liftEncoderState = true;
    private boolean firstPressa = true;
    private boolean firstPressb = true;
    private boolean firstPressx = true;
    private boolean firstPressDown = true;
    private boolean firstPressUp = true;
    private boolean firstLeftRun = true;

    private boolean lastWaffleState = false;
    private boolean isWaffleStateRaised = false;
    private boolean readyToGrab = false;
    private boolean manualReset = false;
    private boolean stoneFullyInStraightener = false;
    private boolean straightenerBusy = false;
    private boolean firstRightRun = true;

    static final double countsPerMotor          = 383.6;
    static final double gearReduction           = 1.0 ;
    static final double wheelDiameter           = 1.771653543307087 ;
    static final double countsPerInch           = (countsPerMotor * gearReduction) / (wheelDiameter * Math.PI);
    static final double liftOffset = (1.5 * countsPerInch);     //TODO Change
    static final double minimumSwingPosition = 8;

    private String stoneOrientation = "empty";
    private String lCurrentPosition = "lDisengage";
    private String rCurrentPosition = "rDisengage";
    private double foundationatorPosition = .335;
    private double orientDistance = 0;
    private double lDisengage = .2;
    private double rDisengage = .9;
    private double lEngage = .85;
    private double rEngage = .25;
    private double startRightTime = 0;
    private double currentRightTime = 0;
    private double actualRightTime = 0;
    private double lastActualRightTime = 0;
    private double rightOrientCheck = 1;
    private double startLeftTime = 0;
    private double currentLeftTime = 0;
    private double actualLeftTime = 0;
    private double lastActualLeftTime = 0;
    private double targetTime = .5;
    private double stoneDistance = 0;


    @Override
    public void init() {
        telemetry.addData("Version Number", "12-26-19 700pm");
        initializeVerticalLift();
        initializeFoundationator();
        initializeGrabber();
        initializeDriveTrain();
        initializeOdometry();
        initializeIntakeMechanism();
        initializeStraightener();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        startFoundationator();
        startIntakeMechanism();
        startOdometry();
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();
    }

    @Override
    public void loop() {
        checkVerticalLift();
        checkFoundationator();
        checkDriveTrain();
        checkOdometry();
        checkIntakeMechanism();
        checkStraightener();
        telemetry.update();
    }

    //
    // VERTICAL LIFT
    //

    public void initializeVerticalLift() {
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setPower(0);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkVerticalLift() {
        double LiftUpButton = gamepad1.right_trigger;
        double LiftDownButton = gamepad1.left_trigger;
        boolean LiftManualToggleButton = gamepad1.y;
        boolean LiftOverideDownButton = gamepad2.dpad_down;
        boolean LiftOverideUpButton = gamepad2.dpad_up;

        if (LiftManualToggleButton) {
            if (firstPressa) {
                liftEncoderState =! liftEncoderState;
                firstPressa = false;
            }
        }
        else {
            firstPressa = true;
        }

        if (LiftOverideDownButton) {
            if (firstPressDown) {
                liftHeight--;
                firstPressDown = false;
            }
        }
        else {
            firstPressDown = true;
        }

        if (LiftOverideUpButton) {
            if (firstPressUp) {
                liftHeight++;
                firstPressUp = false;
            }
        }
        else {
            firstPressUp = true;
        }

        if (liftEncoderState) {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(1);
            telemetry.addData("LiftMotorDistance",LiftMotor.getCurrentPosition() - LiftMotor.getTargetPosition());
            //setting the Target position if we press the right bumper
            if (LiftUpButton > .5) {
                if (firstLiftUpButton) {
                    liftUpCommand = true;
                    firstLiftUpButton = false;
                }
                else {
                    liftUpCommand = false;
                }
            }
            else {
                firstLiftUpButton = true;
            }

            if (LiftDownButton > .5) {
                if (firstLiftDownButton) {
                    liftDownCommand = true;
                    firstLiftDownButton = false;
                }
                else {
                    liftDownCommand = false;
                }
            }
            else {
                firstLiftDownButton = true;
            }

            if (liftUpCommand) {
                if (liftHeight >= 10) {
                    liftHeight = 10;
                }
                else {
                    liftHeight++;
                }

                LiftMotor.setTargetPosition((int)(liftHeight * (4 * countsPerInch) + liftOffset));
                liftUpCommand = false;
            }
            else if (liftDownCommand) {
                LiftMotor.setTargetPosition(0);
                liftDownCommand = false;
            }

            telemetry.addData("Lift Height", liftHeight);

            /*if (liftUpCommand) {
                LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (4 * countsPerInch)));
            }
            //setting the Target position if we press the left Bumper
            else if (liftDownCommand) {
                LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (-4 * countsPerInch)));
            }*/
        }
        else {
            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LiftMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
}

    //
    // FOUNDATIONATOR
    //

    public void initializeFoundationator() {
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
    }

    public void startFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
    }

    public void checkFoundationator() {
        boolean currentWaffleState = gamepad1.a;

        if (lastWaffleState == currentWaffleState){
            return;
        }
        else if(!lastWaffleState && currentWaffleState){
            if(isWaffleStateRaised) {
                lowerFoundationator();
            } else {
                raiseFoundationator();
            }
        }
        lastWaffleState = currentWaffleState;

        if (isWaffleStateRaised) {
            telemetry.addData("Foundationator is", "Raised");
        }
        else {
            telemetry.addData("Foundationator is", "Lowered");
        }
    }

    private void raiseFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
        isWaffleStateRaised = true;
    }

    private void lowerFoundationator() {
        lFoundationator.setPosition(foundationatorPosition);
        rFoundationator.setPosition(0);
        isWaffleStateRaised = false;
    }

    //
    // Grabber Arm
    //

    private void initializeGrabber() {
        GrabberServo =  hardwareMap.servo.get("GrabberServo");
        OrientStoneServo = hardwareMap.servo.get("OrientStoneServo");
        MoveArmServo = hardwareMap.servo.get("MoveArmServo");
    }

    private void grabStoneFromStraightener() {
        GrabberServo.setPosition(.5);
        // close grabber servo
        return;
    }
    //this.moveArmToScorePosition("horizontal");
    private void moveArmToScorePosition(String orientation) {
        if(orientation.contentEquals("long")) {
        }
        else if(orientation.contentEquals("wide")) {
        }
        // move arm to scoring position
        // orient the grabber to prepare to place the stone
        return;
    }

    private void releaseStoneFromGrabber() {
        GrabberServo.setPosition(0);
        // open grabber servo to release stone
        return;
    }

    //
    // DRIVE TRAIN
    //

    public void initializeDriveTrain() {
        RightBackupSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "RightBackupSensor");
        LeftBackupSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "LeftBackupSensor");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkDriveTrain() {
        rightBackupDistance = RightBackupSensor.getDistance(DistanceUnit.INCH);
        leftBackupDistance = LeftBackupSensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("Right Backup Distance", rightBackupDistance);
        telemetry.addData("Left Backup Distance", leftBackupDistance);

        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            if (firstPressBumpers) {
                //autoDrivingStage = 0;
                //autoDrivingTimes = 0;
                firstPressBumpers = false;
            }
            goToPosition(0,0,.5,.5, 0);
        } else {
            firstPressBumpers = true;

            double forwardButton = gamepad1.left_stick_y;
            double sidewaysButton = gamepad1.left_stick_x;
            double spinningButton = gamepad1.right_stick_x;

            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);

            forwardButton = DeadModifier(forwardButton);
            sidewaysButton = DeadModifier(sidewaysButton);
            spinningButton = DeadModifier(spinningButton);

            Drive(forwardButton, sidewaysButton, spinningButton);
        }
    }

    // From Gluten Free

    public void goToPosition(double x, double y, double movementSpeed, double turnSpeed, double preferredAngle) {
        double distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (RobotRotation-90));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - 180 + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle / 30, -1, 1) * turnSpeed;

        if (distanceToTarget < 3) {
            movement_turn = 0;
        }
        Drive(.3*movement_y, movement_x, .3*movement_turn);
        Drive(.3*movement_y, movement_x, .3*movement_turn);
    }

    public double AngleWrap(double angle){
        while(angle < -180) {
            angle += 360;
        }
        while(angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    // From Wizards

    /*private void goToPosition(double targetXPosition, double targetYPosition, double targetOrientation, double power, double allowableDistanceError) {
        double distanceToXTarget = (targetXPosition*COUNTS_PER_INCH) - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = (targetYPosition*COUNTS_PER_INCH) - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

        double robot_movement_x_component = calculateX(robotMovementAngle, power);
        double robot_movement_y_component = calculateY(robotMovementAngle, power);
        double pivotCorrection = targetOrientation - globalPositionUpdate.returnOrientation();

        if (distance < allowableDistanceError) {
            Drive(0,0,0);
        }
        else {
//            FrontRight.setPower(-DZSidewaysButton - DZForwardButton - DZSpinningButton);
//            FrontLeft.setPower(-DZSidewaysButton + DZForwardButton - DZSpinningButton);
//            BackRight.setPower(DZSidewaysButton - DZForwardButton - DZSpinningButton);
//            BackLeft.setPower(DZSidewaysButton + DZForwardButton - DZSpinningButton);

            telemetry.addData("robot_movement_x_component", robot_movement_x_component);
            telemetry.addData("robot_movement_y_component", robot_movement_y_component);
            telemetry.addData("pivotCorrection", pivotCorrection);
            telemetry.addData("robotMovementAngle", robotMovementAngle);
        }
    }*/

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    // By DH

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

    public void smoothDriveToScoringPosition() {
        java.lang.String turningDirection;
        double distanceFromOrientation;

        if (RobotRotation > 2 && RobotRotation < 180) {
            turningDirection = "left";
        } else {
            turningDirection = "right";
        }
        telemetry.addData("TurningDirection", turningDirection);

        if (autoDrivingStage == 0) {
            if (RobotRotation > 5 && RobotRotation < 355) {
                if (turningDirection.equals("right")) {
                    distanceFromOrientation = 360 - RobotRotation;
                    if (distanceFromOrientation > 120) {
                        Drive(0, 0, .6);
                    } else if (distanceFromOrientation > 45) {
                        Drive(0, 0, .35);
                    } else {
                        Drive(0, 0, .15);
                    }
                } else {
                    distanceFromOrientation = RobotRotation;
                    if (distanceFromOrientation > 120) {
                        Drive(0, 0, -.6);
                    } else if (distanceFromOrientation > 45) {
                        Drive(0, 0, -.35);
                    } else {
                        Drive(0, 0, -.15);
                    }
                }
                telemetry.addData("DistanceFromOrientation", distanceFromOrientation);
            }
            else {
                autoDrivingStage++;
            }
        }
        else if (autoDrivingStage == 1) {
            if (RobotYPosition < -.5 || RobotYPosition > .5) {
                if (RobotYPosition < 0) {
                    if (RobotYPosition < -12) {
                        Drive(-.3, 0, 0);
                    } else {
                        Drive(-.1, 0, 0);
                    }
                } else {
                    if (RobotYPosition > 12) {
                        Drive(.3, 0, 0);
                    } else {
                        Drive(.1, 0, 0);
                    }
                }
            }
            else {
                autoDrivingStage++;
            }
        }
        else if (autoDrivingStage == 2) {
            if (RobotXPosition < -.5 || RobotXPosition > .5) {
                if (RobotXPosition < 0) {
                    if (RobotXPosition < -6) {
                        Drive(0, .5, 0);
                    } else {
                        Drive(0, .2, 0);
                    }
                } else {
                    if (RobotXPosition > 6) {
                        Drive(0, -.5, 0);
                    } else {
                        Drive(0, -.2, 0);
                    }
                }
            } else {
                if (autoDrivingTimes == 0) {
                    autoDrivingTimes = 1;
                    autoDrivingStage = 0;
                }
                else {
                    Drive(0, 0, 0);
                }
            }
        }
    }

    public void oneMotionDriveToScoringPosition() {
        java.lang.String turningDirection;
        double distanceFromOrientation;
        int inversingPowerRotationMultiplier;

        if (RobotRotation > 2 && RobotRotation < 180) {
            turningDirection = "left";
        } else {
            turningDirection = "right";
        }
        telemetry.addData("TurningDirection", turningDirection);
        if (turningDirection.equals("right")) {
            distanceFromOrientation = 360 - RobotRotation;
            inversingPowerRotationMultiplier = 1;
        }
        else {
            distanceFromOrientation = RobotRotation;
            inversingPowerRotationMultiplier = -1;
        }
        telemetry.addData("DistanceFromOrientation", distanceFromOrientation);
    }

//    public double DeadModifier(double joystickValue) {
//        forwardButton = DeadModifier(forwardButton);
//        sidewaysButton = DeadModifier(sidewaysButton);
//        spinningButton = DeadModifier(spinningButton);
//
//        Drive(forwardButton, sidewaysButton, spinningButton);
//    }

    // By Paul

    public double  DeadModifier(double joystickValue) {
        if(joystickValue < DEADZONE && joystickValue > -DEADZONE)
            return 0;
        else {
            return joystickValue;
        }
    }

    public void Drive(double DZForwardButton, double DZSidewaysButton, double DZSpinningButton) {
        DZSpinningButton = -DZSpinningButton;
        FrontRight.setPower(-DZSidewaysButton - DZForwardButton + DZSpinningButton);
        FrontLeft.setPower(-DZSidewaysButton + DZForwardButton + DZSpinningButton);
        BackRight.setPower(DZSidewaysButton - DZForwardButton + DZSpinningButton);
        BackLeft.setPower(DZSidewaysButton + DZForwardButton + DZSpinningButton);
    }

    //
    // Odometry
    //

    public void initializeOdometry() {
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //StartingXPosition = Double.parseDouble(ReadWriteFile.readFile(startingXpositionFile).trim());
        //StartingYPosition = Double.parseDouble(ReadWriteFile.readFile(startingYpositionFile).trim());
        //StartingRotation = Double.parseDouble(ReadWriteFile.readFile(startingθpositionFile).trim());
        StartingXPosition = 0;
        StartingYPosition = 0;
        StartingRotation = 0;
    }

    public void startOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    public void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;

        if (gamepad1.b) {
            if (firstPressb) {
                firstPressb = false;
                StartingXPosition = -RobotXPosition;
                StartingYPosition = -RobotYPosition;
                StartingRotation = -RobotRotation;
            }
        }
        else {
            firstPressb = true;
        }

        if (RobotRotation < 0){
            RobotRotation += 360;
        }

        telemetry.addData("X", RobotXPosition);
        telemetry.addData("Y", RobotYPosition);
        telemetry.addData("θ", RobotRotation);

        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("θ (Degrees)", globalPositionUpdate.returnOrientation());

        telemetry.addData("Vertical Left Encoder", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Encoder", verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", horizontal.getCurrentPosition());

        telemetry.addData("Thread Active", positionThread.isAlive());
    }

    double getDistanceFromCoordinates(double x, double y, OdometryGlobalCoordinatePosition position) {
        double deltaX = x - position.returnXCoordinate();
        double deltaY = y - position.returnYCoordinate();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    //
    // INTAKE
    //

    public void initializeIntakeMechanism() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeAssistServo = hardwareMap.crservo.get("IntakeAssistServo");
        IntakeReleaseServo = hardwareMap.servo.get("IntakeReleaseServo");
        IntakeMotor.setPower(0);
        IntakeAssistServo.setPower(0);
    }
    public void startIntakeMechanism() {
        IntakeReleaseServo.setPosition(.6);
    }

    public void checkIntakeMechanism() {
        intakeButton = gamepad1.dpad_up;
        outputButton = gamepad1.dpad_down;

        intake(intakeButton);

        if (gamepad1.x) {
            if (firstPressx) {
                if (intakeReleaseState == 1) {
                    intakeReleaseState = 0;
                }
                else {
                    intakeReleaseState = 1;
                }
                firstPressx = false;
            }
        }
        else {
            firstPressx = true;
        }

        if (intakeReleaseState == 0) {
            IntakeReleaseServo.setPosition(.15);
        }
        else {
            IntakeReleaseServo.setPosition(.6);
        }


    }

    public void intake(boolean intakeButton) {
        if (intakeButton) {
            if (firstPressDpadUp) {
                if (intakeState == 1) {
                    intakeState = 0;
                }
                else {
                    intakeState = 1;
                }
                firstPressDpadUp = false;
            }
        }
        else {
            firstPressDpadUp = true;
        }
        if (outputButton) {
            IntakeMotor.setPower(-1);
            IntakeAssistServo.setPower(1);
        }
        else {
            if (intakeState == 1) {
                IntakeMotor.setPower(1);
                IntakeAssistServo.setPower(-1);
            }
            else if (intakeState == 0) {
                IntakeMotor.setPower(0);
                IntakeAssistServo.setPower(0);
            }
        }
    }

    //
    // Straightener
    //

    public void initializeStraightener() {
        OrientationSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "OrientationSensor");
        StonePresenceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "StonePresenceSensor");
        OrientationServoLeft = hardwareMap.get(Servo.class, "OrientationServoLeft");
        OrientationServoRight = hardwareMap.get(Servo.class, "OrientationServoRight");
        OrientationServoLeft.setPosition(lDisengage);
        OrientationServoRight.setPosition(rDisengage);
    }

    public void checkStraightener() {
        stoneDistance = StonePresenceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Stone Distance", stoneDistance);
        if (stoneDistance < 1.5) {
            stoneFullyInStraightener = true;
        }
        else {
            stoneFullyInStraightener = false;
        }

        orientStone();
        manualOverride();
    }

    private void senseOrientation() {
        orientDistance = OrientationSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Orient Distance", orientDistance);

        // StoneOrientation is assigned relative to the side of the robot that the studs of the stone
        // are on when looking at the robot from the back.
        // this is true for the servos as well; left and right are assigned relative to the back
        if(!straightenerBusy) {
            if (orientDistance > .25 && orientDistance <= .75) {
                stoneOrientation = "left";
            } else if (orientDistance > .75 && orientDistance <= 2.3) {
                stoneOrientation = "center";
            } else if (orientDistance > 4) {
                stoneOrientation = "empty";
            } else {
                stoneOrientation = "right";
            }
            telemetry.addData("orientPosition:  ", stoneOrientation);
        }
    }

    private void orientStone() {
        senseOrientation();

        if (stoneFullyInStraightener) {
            if (stoneOrientation.equals("right")) {
                runLeftServo();
            }

            if (stoneOrientation.equals("left")) {
                runRightServo();
            }

            if (stoneOrientation.equals("center")) {
                readyToGrab = true;
            }
            else {
                readyToGrab = false;
            }
        }
    }

    private void manualOverride() {
        if (gamepad2.right_bumper) {
            manualReset = true;
        }
        if (gamepad2.dpad_right) {
            runLeftServo();
        }
        if (gamepad2.dpad_left) {
            runRightServo();
        }
    }

    private void runRightServo() {
        if(firstRightRun && !straightenerBusy) {
            OrientationServoRight.setPosition(rEngage);
            startRightTime = getRuntime();
            firstRightRun = false;
            straightenerBusy = true;
        }
        currentRightTime = getRuntime();
        actualRightTime = (currentRightTime-startRightTime);
        telemetry.addData("actualRightTime", actualRightTime);
        telemetry.addData("target time", targetTime);

        if(actualRightTime > targetTime) {
            OrientationServoRight.setPosition(rDisengage);
            telemetry.addData("is ready for return", "yes");
            firstRightRun = true;
            straightenerBusy = false;
        }
    }

    private void runLeftServo() {
        if(firstLeftRun && !straightenerBusy) {
            OrientationServoLeft.setPosition(lEngage);
            startLeftTime = getRuntime();
            firstLeftRun = false;
            straightenerBusy = true;
        }
        currentLeftTime = getRuntime();
        actualLeftTime = (currentLeftTime-startLeftTime);
        telemetry.addData("actualLeftTime", actualLeftTime);
        telemetry.addData("target time", targetTime);

        if(actualLeftTime > targetTime) {
            OrientationServoLeft.setPosition(lDisengage);
            telemetry.addData("is ready for return", "yes");
            firstLeftRun = true;
            straightenerBusy = false;
        }
    }
}