package frc.robot.utilities;

public record AABB(double x, double y, double width, double height) {
    public boolean intersectingPoint(double otherX, double otherY) {
        return otherX < x + width && otherX > x - width && otherY < x + width && otherY > x - width;
    }

    public boolean intersectingAABB(AABB other) {
        return ((other.x + other.width < x + width && other.x + other.width > x - width)
                        || (other.x - other.width < x + width && other.x - other.width > x - width))
                && ((other.y + other.height < y + height && other.y + other.height > y - height)
                        || (other.y - other.height < y + height && other.y - other.height > y - height));
    }
}
