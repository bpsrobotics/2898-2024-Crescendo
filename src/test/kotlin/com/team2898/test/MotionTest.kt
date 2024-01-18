package com.team2898.test

import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import java.util.LinkedHashMap

class MotionTest {
    private var lmap = LinkedHashMap<Int, Int>();

    @Test
    fun whenPut2_thenGet2() {
        lmap.put(1,2);
        Assertions.assertEquals(2, lmap.get(1), "The value for key 1 should be 2")
    }

    @Test
    fun whenAdd2_thenSizeIs2() {
        lmap.put(1,2);
        lmap.put(2,3);
        Assertions.assertEquals(2, lmap.size)
        // Check that values are in insertion order
        val values = lmap.values
        Assertions.assertEquals( values.elementAt(0), 2);
        Assertions.assertEquals( values.elementAt(1), 3);
    }
}