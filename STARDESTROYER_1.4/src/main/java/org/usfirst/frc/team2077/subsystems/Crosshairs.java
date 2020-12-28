/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usfirst.frc.team2077.Robot.*;

public class Crosshairs extends SubsystemBase {


    private static double MINIMUM_RANGE = 18; // inches // TODO: Configure in Constants;
    private static double MAXIMUM_RANGE = 54*12; // inches // TODO: Configure in Constants;

    private double azimuth_ = 0;
    private double range_ = 50*12; // TODO: Don't rely on these initial values.

    public Crosshairs() {} // TODO: Remove other constructors, then remove this one.

    public final void set(double azimuth, double range) {
        azimuth_ = normalize(azimuth);
        range_ = constrain(range);
    }

    public double[] get() {
        return new double[] {azimuth_, range_};
    }

    public double getAzimuth() {
        return azimuth_;
    }

    public double getRange() {
        return range_;
    }

    private static double constrain(double range) {
        return Math.max(robot_.constants_.MINIMUM_TARGET_RANGE, Math.min(robot_.constants_.MAXIMUM_TARGET_RANGE, range));
    }

    private static double normalize(double azimuth) {
        return (((azimuth % 360) + 360 + 180) % 360) - 180;
    }

    @Override
    public String toString() {
        return "Azimuth:" + Math.round(azimuth_*10.)/10. + " degrees, Range:" + Math.round(range_*10.)/10. + " inches";
    }

    /**
     * Test code.
     * May be run locally in VSCode.
     */
    public static void main(String[] argv) {

        System.out.println("" + normalize(361) + " (1);");
        System.out.println("" + normalize(-361) + " (-1);");
        System.out.println("" + normalize(359) + " (-1);");
        System.out.println("" + normalize(-359) + " (1);");
        System.out.println("" + normalize(181) + " (-179);");
        System.out.println("" + normalize(-181) + " (179);");
        System.out.println("" + normalize(179) + " (179);");
        System.out.println("" + normalize(-179) + " (-179);");
    }








    private int pixelWidth_;
    private int pixelHeight_;
    private int pixelFocalLength_;
    private double cameraNorth_;
    private double cameraHeight_;
    private double cameraTilt_;
    private double targetHeight_;
    private double horizontalFOV_;

    private double pixelX_ = 0;
    private double pixelY_ = 0;

    /**
     * Support for alternate practice target.
     * @param cameraID Unique camera identifier for display on drive station. // TODO: Remove.
     * @param pixelWidth Camera image width.
     * @param pixelHeight Camera image height.
     * @param pixelFocalLength Camera focal length in pixels.
     * @param cameraNorth Forward distance from chassis center to camera lens, in inches.
     * @param cameraHeight Height of camera lens above floor, in inches.
     * @param cameraTilt Upward tilt of camera axis in degrees.
     * @param targetHeight Target height in inches. // TODO: Remove.
     */
    public Crosshairs(String cameraID, int pixelWidth, int pixelHeight, int pixelFocalLength, 
        double cameraNorth, double cameraHeight, double cameraTilt, double targetHeight) {

        pixelWidth_ = pixelWidth;
        pixelHeight_ = pixelHeight;
        pixelFocalLength_ = pixelFocalLength;
        cameraNorth_ = cameraNorth;
        cameraHeight_ = cameraHeight;
        cameraTilt_ = cameraTilt;
        targetHeight_ = targetHeight; // TODO: Use constants.
        horizontalFOV_ = 2. * Math.toDegrees(Math.atan2(pixelWidth_/2., pixelFocalLength_));
    }

    public double getHorizontalFOV() {
        return horizontalFOV_;
    }

    public double[] getCamera() {
        targetToCamera();
        return new double[] {pixelX_, pixelY_, pixelWidth_, pixelHeight_};
    }

    private void targetToCamera() {

        double azimuth = azimuth_;
        double range = range_;
        double north = range_ * Math.cos(Math.toRadians(azimuth_));
        double east = range_ * Math.sin(Math.toRadians(azimuth_));

        north -= cameraNorth_;
        azimuth = Math.toDegrees(Math.atan2(east, north)); // from camera to target
        range = Math.sqrt(north*north + east*east); // from camera to target
        double elevation = Math.toDegrees(Math.atan2(targetHeight_ - cameraHeight_, range)); // from ground to target
        elevation -= cameraTilt_; // from camera axis to target
        pixelX_ = pixelFocalLength_ * Math.tan(Math.toRadians(Math.max(-horizontalFOV_/2., Math.min(horizontalFOV_/2., azimuth))));
        //pixelY_ = pixelFocalLength_ * Math.tan(Math.toRadians(elevation)); // perspective projection
        pixelY_ = elevation / 0.13; // horizontal cylindrical projection, 2/25 version
    }
}
