package org.firstinspires.ftc.teamcode.lib.recording;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;


public class InputManager {

    public BufferedWriter bufferedWriter;
    public FileWriter fileWriter;
    public FileReader fileReader;
    public BufferedReader bufferedReader;

    double[] leftFrontValues;
    double[] leftBackValues;
    double[] rightFrontValues;
    double[] rightBackValues;

    //double ly, rx, lx;

    //int lTicks, rTicks, sTicks;

    public enum DriveType{
        h4, h3, norm4, norm2
    }

    boolean useGyro, useCV;

    int numDriveMotors, numAuxMotors, numServos;

    int m1Ticks = 0, m2Ticks = 0, m3Ticks = 0, m4Ticks = 0, m5Ticks = 0, m6Ticks = 0, m7Ticks = 0, m8Ticks = 0;

    int s1Pos, s2Pos, s3Pos, s4Pos, s5Pos, s6Pos, s7Pos, s8Pos, s9Pos, s10Pos, s11Pos, s12Pos;

    int gyroPos;

    int countLines, countReplays;

    public InputManager(DriveType type, int numAuxMotors, int numServos){

        switch (type){
            case h4:{

                numDriveMotors = 4;

            }
            case h3:{

                numDriveMotors = 3;

            }
            case norm4:{

                numDriveMotors = 4;

            }
            case norm2:{

                numDriveMotors = 2;

            }
        }

        this.numAuxMotors = numAuxMotors;
        this.numServos = numServos;

    }

    public void setupRecording(File file){

        // Assume default encoding.
        fileWriter = null;

        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Always wrap FileWriter in BufferedWriter.
        bufferedWriter = new BufferedWriter(fileWriter);

    }

    public void writeInputs(//Robot robot
    int i
    ){

        try {
            //bufferedWriter.write(m1Ticks + ";" + m2Ticks + ";" + m3Ticks + ";" + m4Ticks + ";" + m5Ticks + ";" + m6Ticks + ";" + m7Ticks + ";" + m8Ticks + ";" + s1Pos + ";" + s2Pos + ";" + s3Pos + ";" + s4Pos + ";" + s5Pos + ";" + s6Pos + ";" + s7Pos + ";" + s8Pos + ";" + s9Pos + ";" + s10Pos + ";" + s11Pos + ";" + s12Pos + ";" + gyroPos);
            bufferedWriter.write(i + " " + i);

            bufferedWriter.newLine();
            System.out.println(i);
        } catch (IOException e) {
            e.printStackTrace();
        }

        //dt.manualDrive(gamepad1);

    }

    public void setupPlayback(File file){
        String[] tempString;
        String line;

        leftFrontValues = new double[50000];
        rightFrontValues = new double[50000];

        try {

            fileReader = new FileReader(file);
            bufferedReader = new BufferedReader(fileReader);

            while ((line = bufferedReader.readLine()) != null) {

                tempString = line.split(";");

                m1Ticks = Integer.parseInt(tempString[0]);
                m2Ticks = Integer.parseInt(tempString[1]);
                m3Ticks = Integer.parseInt(tempString[2]);
                m4Ticks = Integer.parseInt(tempString[3]);
                m5Ticks = Integer.parseInt(tempString[4]);
                m6Ticks = Integer.parseInt(tempString[5]);
                m7Ticks = Integer.parseInt(tempString[6]);
                m8Ticks = Integer.parseInt(tempString[7]);

                gyroPos = Integer.parseInt(tempString[19]);

                countLines++;
            }

            bufferedReader.close();

            countReplays = 0;
        }
        catch(IOException ex) {
            ex.printStackTrace();
        }
    }

    public void replayInputs(){
        while(countReplays <= countLines) {
            //dt.runMotorsIndiv(leftFrontValues[countReplays], rightFrontValues[countReplays], leftBackValues[countReplays], rightBackValues[countReplays]);
            countReplays++;
        }
    }

    public void stopAndReset() {

        try {

            bufferedReader.close();
            bufferedWriter.close();

        } catch (IOException e) {
            e.printStackTrace();
        }

    }

}