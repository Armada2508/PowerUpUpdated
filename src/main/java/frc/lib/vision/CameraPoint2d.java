package frc.lib.vision;

public class CameraPoint2d {
    private double m_x;
    private double m_y;
    private boolean m_isAngle;
    private FOV m_fov;
    private Resolution m_res;

    /**
     * Creates a new CameraPoint2d
     * 
     * @param tx The x angle of the target relative to the camera
     * @param ty The y angle of the target relative to the camera
     */
    public CameraPoint2d(double tx, double ty) {
        this(tx, ty, true);
    }
    
    /**
     * Creates a new CameraPoint2d
     * 
     * @param x The x coordinate
     * @param y The y coordinate
     * @param angle If these CameraPoint2d values are angles
     */
    public CameraPoint2d(double x, double y, boolean angle) {
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
     * Check if the CameraPoint2d is an angle
     * @return If the CameraPoint2d is an angle
     */

    public boolean isAngle() {
        return m_isAngle;
    }

    /**
     * Center the CameraPoint2d
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
     * Converts the CameraPoint2d to an angle
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
     * Converts the CameraPoint2d to pixel coordinates
     * @param fov The field-of-view of the camera
     * @param resolution The resolution of the camera
     */
    public void toPixels(FOV fov, Resolution resolution) {
        if(m_isAngle) {
            m_x = VisionUtil.anglesToPixels(m_x, fov.getX(), resolution.getX());
            m_y = VisionUtil.anglesToPixels(m_y, fov.getY(), resolution.getY());
        }
    }


    /**
     * Stores camera parameters for point
     * @param fov
     * @param resolution
     */
    public void config(FOV fov, Resolution resolution) {
        m_fov = fov;
        m_res = resolution;
    }

    
    /**
     * Converts the CameraPoint2d to an angle using the FOV and Resolution set globally with {@link CameraPoint2d#config(FOV, Resolution)}
     */
    public void toAngle() {
        if(m_fov == null || m_res == null) {
            return;
        }
        if(!m_isAngle) {
            m_x = VisionUtil.pixelsToAngles(m_x, m_fov.getX(), m_res.getX());
            m_y = VisionUtil.pixelsToAngles(m_y, m_fov.getY(), m_res.getY());
        }
    }

    
    /**
     * Converts the CameraPoint2d to pixel coordinates using the FOV and Resolution set globally with {@link CameraPoint2d#config(FOV, Resolution)}
     */
    public void toPixels() {
        if(m_fov == null || m_res == null) {
            return;
        }
        if(m_isAngle) {
            m_x = VisionUtil.anglesToPixels(m_x, m_fov.getX(), m_res.getX());
            m_y = VisionUtil.anglesToPixels(m_y, m_fov.getY(), m_res.getY());
        }
    }

}