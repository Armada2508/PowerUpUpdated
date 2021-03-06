package frc.lib.vision;

public class VisionUtil {
    /**
     * Converts a pixel input to an angle output(measured in degrees)
     * 
     * @param pixel The pixel input
     * @param fov The full field of view of the camera in the desired direction
     * @param resolution The resolution of the camera in the desired direction
     * @return The angle in degrees that the input is relative to the center camera
     */
    public static double pixelsToAngles(double pixel, double fov, double resolution) {
        return Math.toDegrees(Math.atan(Math.tan(Math.toRadians(fov/2.0))*pixel/(resolution/2.0)));
    }
    /**
     * Converts an angle input to a pixel output
     * 
     * @param degrees The angle input(measured in degrees)
     * @param fov The full field of view of the camera in the desired direction
     * @param resolution The resolution of the camera in the desired direction
     * @return The pixel value at which the specified angle will be found
     */
    public static double anglesToPixels(double degrees, double fov, double resolution) {
        return Math.tan(Math.toRadians(degrees))/Math.tan(Math.toRadians(fov/2.0))*(resolution/2.0);
    }

    /**
     * Convert pixels to normalized coordinates(-1 to +1) from a centered pixel coordinate system(e.g. -159.5-159.5 if there were 320 pixels horizontally)
     * 
     * @param pixel The pixel value to convert
     * @param resolution The resolution of the camera in the desired direction
     * 
     * @return The normalized coordinate
     * 
     * @see centerPixels()
     */
    public static double normalizePixels(int pixel, double resolution) {
        return pixel / (resolution / 2.0);
    }

    /**
     * Convert pixels to centered coordinates(e.g. 159.5 to +159.5 for 320 resolution) from a pixel coordinate system(e.g. 0-319 for 320 resolution)
     * 
     * @param pixel The pixel value to convert
     * @param resolution The resolution of the camera in the desired direction
     * @param inverted If the positive and negative directions are flipped
     * @return The centered coordinate
     */
    public static double centerPixels(int pixel, double resolution, boolean inverted) {
        if(inverted) {
            return (((resolution / 2.0) - 0.5) - (double)pixel);
        } else {
            return ((double)pixel - ((resolution / 2.0) - 0.5));
        }
    }

    /**
     * Convert angles to normalized coordinates(-1 to +1) from a centered angle coordinate system(e.g -30 degrees to +30 degrees)
     * @param angle The input angle
     * @param fov The full field of view of the camera in the desired direction
     * @return The normalized angle coordinate
     */
    public static double normalizeAngle(double angle, double fov) {
        return angle / (fov / 2.0);
    }
}