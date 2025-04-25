package frc.robot.utilities;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.jupiter.api.Test;

public class AABBTest {

    @Test
    public void simple() {
        AABB a = new AABB(0, 0, 0.5, 0.5);
        AABB b = new AABB(0.9, 0, 0.5, 0.5);
        assertTrue(a.intersectingAABB(b));
        assertTrue(b.intersectingAABB(a));

        AABB c = new AABB(1.1, 0, 0.5, 0.5);
        assertFalse(a.intersectingAABB(c));
        assertFalse(c.intersectingAABB(a));

        AABB d = new AABB(0.9, 0, 0.5, 0.5);
        assertTrue(a.intersectingAABB(d));
        assertTrue(d.intersectingAABB(a));

        AABB e = new AABB(1.1, 0, 0.5, 0.5);
        assertFalse(a.intersectingAABB(e));
        assertFalse(e.intersectingAABB(a));
    }

    @Test
    public void differentSizes() {
        AABB a = new AABB(0, 0, 1, 1);
        AABB b = new AABB(0, 0, 0.5, 0.5);
        assertTrue(a.intersectingAABB(b));

        AABB c = new AABB(0, 0, 0.5, 2);
        assertTrue(a.intersectingAABB(c));

        AABB e = new AABB(1.6, 0, 0.5, 0.5);
        assertFalse(a.intersectingAABB(e));

        AABB d = new AABB(0, 7.6, 0.5, 0.5);
        assertFalse(a.intersectingAABB(d));
    }
}
