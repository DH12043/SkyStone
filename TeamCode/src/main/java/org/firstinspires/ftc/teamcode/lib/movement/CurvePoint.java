package org.firstinspires.ftc.teamcode.lib.movement;


import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.autoMoveSpeed;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.autoTurnSpeed;

public class CurvePoint {
    public double x, y, moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance,
                      double slowDownTurnRadians, double slowDownTurnAmount){

        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;

    }

    public CurvePoint(double x, double y, double followDistance,
                      double slowDownTurnRadians, double slowDownTurnAmount){

        this.x = x;
        this.y = y;
        this.moveSpeed = autoMoveSpeed;
        this.turnSpeed = autoTurnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;

    }

    public CurvePoint(CurvePoint thisPoint){

        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        pointLength = thisPoint.pointLength;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;

    }

    public Point toPoint(){
        return new Point(x,y);
    }


    public void setPoint(Point point) {

        x = point.x;
        y = point.y;

    }
}
