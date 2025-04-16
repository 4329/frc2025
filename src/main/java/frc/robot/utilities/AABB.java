package frc.robot.utilities;

public record AABB(double x, double y, double width, double height) {

    private static boolean checkPoint1D(double point, double x, double width) {
        return point < x + width && point > x - width;
    }

    private static boolean checkIntersects(AABB a, AABB b) {
        return (checkPoint1D(b.x - b.width, a.x, a.width) || checkPoint1D(b.x + b.width, a.x, a.width) || checkPoint1D(b.x, a.x, a.width))
                && (checkPoint1D(b.y - b.height, a.y, a.height) || checkPoint1D(b.y + b.height, a.y, a.height) || checkPoint1D(b.y, a.y, a.height));
    }

    public boolean intersectingAABB(AABB other) {
        return checkIntersects(this, other) || checkIntersects(other, this);
    }
}
