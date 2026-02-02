package frc.lib.geometry;

public class Geometry {
    public static class Vector3D
    {
        public double x = 0;
        public double y = 0;
        public double z = 0;
        
        public Vector3D(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        // When applying a rotation to a vector, apply it in the same order here: vector.rotRoll().rotPitch().rotYaw()
        public Vector3D rotRoll(double angle) {
            double z = this.z;
            double x = this.x;
            double y = this.y;
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            return new Vector3D(x*cos-y*sin, x*sin+y*cos, z);
        }
        public Vector3D rotPitch(double angle) {
            double z = this.z;
            double x = this.x;
            double y = this.y;
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            return new Vector3D(x, y*cos-z*sin, y*sin+z*cos);
        }
        public Vector3D rotYaw(double angle) {
            double z = this.z;
            double x = this.x;
            double y = this.y;
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            return new Vector3D(x*cos+z*sin, y, -x*sin+z*cos);
        }
        
    }
    public static class Point3D
    {
        /*
         * +x is forward
         * +y is rightward
         * +z is upward
         */
        Vector3D position = new Vector3D(0, 0, 0);

        /*
        * +x is clockwise yaw
        * +y is upwards pitch
        * +z is leftwards tilt
        */
        Vector3D rotation = new Vector3D(0, 0, 0);

        public Point3D(Vector3D position, Vector3D rotation)
        {
            this.position = position;
            this.rotation = rotation;
        }
        public void setRotation(Vector3D rotation)
        { // Modulates rotation to be between 0 and 6.28
            this.rotation = new Vector3D((rotation.x%(Math.PI*2)), (rotation.y%(Math.PI*2)), (rotation.z%(Math.PI*2)));
        }
        public void setPitch(double pitch)
        {
            this.rotation.y = (pitch%(Math.PI*2));
        }
        public void setYaw(double yaw)
        {
            this.rotation.x = (yaw%(Math.PI*2));
        }
        public void setRoll(double roll)
        {
            this.rotation.z = (roll%(Math.PI*2));
        }

        public Vector3D pointTowards(Vector3D destination)
        {
            double yaw = Math.atan2(destination.y-this.position.y,destination.x-this.position.x);
            double pitch = Math.atan2(destination.z-this.position.z,Math.sqrt(Math.pow(destination.x-this.position.x,2)+Math.pow(destination.y-this.position.y,2)));
            return new Vector3D(yaw, pitch, 0);
        }
        public Vector3D pointTowards(Point3D destination)
        {
            return pointTowards(destination.position);
        }
    }
}
