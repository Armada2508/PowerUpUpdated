package frc.lib.vision;

public class CoordinatePair {
    private double m_x;
    private double m_y;
    private boolean m_isAngle;
    private FOV m_fov;
    private Resolution m_res;

    /**
     * Creates a new CoordinatePair
     * 
     * @param tx The x angle of the target relative to the camera
     * @param ty The y angle of the target relative to the camera
     */
    public CoordinatePair(double tx, double ty) {
        this(tx, ty, true);
    }
    
    /**
     * Creates a new CoordinatePair
     * 
     * @param x The x coordinate
     * @param y The y coordinate
     * @param angle If these CoordinatePair values are angles
     */
    public CoordinatePair(double x, double y, boolean angle) {
        if(angle) {
            m_isAngle = angle;
            m_x = x;
            m_y = y;
        }
    }

    /**
     * Get the x coordinate
     * @return The x coordinate
     */
    public double getX() {
        return m_x;
    }

    /**
     * Get the y coordinate
     * @return The y coordinate
     */

    public double getY() {
        return m_y;
    }

    /**
     * Check if the CoordinatePair is an angle
     * @return If the CoordinatePair is an angle
     */

    public boolean isAngle() {
        return m_isAngle;
    }

    /**
     * Center the CoordinatePair
     * 
     * @param resolution The resolution of the camera
     * @param xInverted If the x coordinate is inverted
     * @param yInverted If the y coordinate is inverted
     */
    public void center(Resolution resolution, boolean xInverted, boolean yInverted) {
        if(!m_isAngle) {
            m_x = VisionUtil.centerPixels((int)m_x, (double)resolution.getX(), xInverted);
            m_y = VisionUtil.centerPixels((int)m_y, (double)resolution.getY(), yInverted);
        } else {
            System.out.println("Pair is not in pixels");
        }
    }

    /**
     * Center the CoordinatePair
     * 
     * @param fov The field-of-view of the camera
     * @param xInverted If the x coordinate is inverted
     * @param yInverted If the y coordinate is inverted
     */
    public void center(FOV fov, boolean xInverted, boolean yInverted) {
        if(!m_isAngle) {
            m_x = VisionUtil.centerPixels((int)m_x, (double)fov.getX(), xInverted);
            m_y = VisionUtil.centerPixels((int)m_y, (double)fov.getY(), yInverted);
        } else {
            System.out.println("Pair is not in pixels");
        }
    }


    /**
     * Converts the CoordinatePair to an angle
     * @param fov The field-of-view of the camera
     * @param resolution The resolution of the camera
     */
    public void toAngle(FOV fov, Resolution resolution) {
        if(!m_isAngle) {
            m_x = VisionUtil.pixelsToAngles(m_x, fov.getX(), resolution.getX());
            m_y = VisionUtil.pixelsToAngles(m_y, fov.getY(), resolution.getY());
        }
    }

    
    /**
     * Converts the CoordinatePair to an angle
     * @param fov The field-of-view of the camera
     * @param resolution The resolution of the camera
     */
    public void toPixels(FOV fov, Resolution resolution) {
        if(m_isAngle) {
            m_x = VisionUtil.anglesToPixels(m_x, fov.getX(), resolution.getX());
            m_y = VisionUtil.anglesToPixels(m_y, fov.getY(), resolution.getY());
        }
    }
}