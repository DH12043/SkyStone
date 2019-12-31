package org.firstinspires.ftc.teamcode.lib.movement;



import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.util.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.lib.util.MathFunctions.lineCircleIntersection;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.*;


public class RobotMovement {

    public static boolean followCurve(ArrayList<CurvePoint> allPoints, double followAngle){


        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);


        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle,
                followMe.turnSpeed);

        boolean ret = false;

        if((worldYPosition >= 95 && worldYPosition <= 105) && (worldXPosition >= 275 && worldXPosition <= 285)){
            ret = true;
        }

        return ret;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> points, Point robotPos, double followRadius){
        CurvePoint followMe = new CurvePoint(points.get(0));

        for(int i = 0; i < points.size()-1; i ++){
            CurvePoint startLine = points.get(i);
            CurvePoint endLine = points.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotPos, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;

            for(Point thisInter : intersections){
                double angle = Math.atan2(thisInter.y - worldYPosition, thisInter.x - worldXPosition);
                double deltaAngle = Math.abs(AngleWrap(angle - worldAngle_rad));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisInter);
                }
            }


        }

        return followMe;

    }

    /**
     *
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double turnSpeed, double preferredAngle){

        double distToTarget = Math.hypot(x-worldXPosition, y- worldYPosition);

        double absoluteAngleToTarget  = Math.atan2(y-worldYPosition, x-worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXPoint = Math.cos(relativeAngleToPoint) * distToTarget;
        double relativeYPoint = Math.sin(relativeAngleToPoint) * distToTarget;

        double movementXPower = relativeXPoint / (Math.abs(relativeXPoint) + Math.abs(relativeYPoint));
        double movementYPower = relativeYPoint / (Math.abs(relativeXPoint) + Math.abs(relativeYPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;


        if(distToTarget < 10){
            movement_turn = 0;
        } else {
            movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;
        }

    }


    public static void manualControl(Gamepad gamepad){

        movement_x = Range.clip(gamepad.left_stick_x, -1, 1);
        movement_y = Range.clip(gamepad.left_stick_y, -1, 1);
        movement_turn = Range.clip(gamepad.right_stick_x, -1, 1);

    }

}
