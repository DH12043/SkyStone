package org.firstinspires.ftc.teamcode.lib.movement;

public class Position{

    private Point current, target;

    double x, y, rot;

    public Position(Point current){

        this.current = current;

    }


    public Position(Point current, Point target){
        this.current = current;
        this.target = target;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getRot(){
        return rot;
    }

    public Point getCurrent(){
        return current;
    }

    public Point getTarget(){
        return target;
    }

    public void setCurrent(Point point){

        current = point;

    }

    public void setTarget(Point point){

        target = point;

    }

    public Point getMovements(){
        if(target.getX() > current.getX()){
            x = target.getX() - current.getX();
        } else if(target.getX() < current.getX()){
            x = -(current.getX() - target.getX());
        } else {
            x = 0;
        }
        if(target.getY() > current.getY()){
            y = target.getY() - current.getY();
        } else if(target.getY() < current.getY()){
            y = -(current.getY() - target.getY());
        } else {
            y = 0;
        }

        return new Point(x,y);
    }

    public Point getMovements(Point current, Point target){
        if(target.getX() > current.getX()){
            x = target.getX() - current.getX();
        } else if(target.getX() < current.getX()){
            x = -(current.getX() - target.getX());
        } else {
            x = 0;
        }
        if(target.getY() > current.getY()){
            y = target.getY() - current.getY();
        } else if(target.getY() < current.getY()){
            y = -(current.getY() - target.getY());
        } else {
            y = 0;
        }

        return new Point(x,y);
    }


    public String toString(){

        return ("(" + current.getX() + ", " + current.getY() + ")");
    }


}