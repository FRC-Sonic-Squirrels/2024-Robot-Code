package frc.lib.team2930;

import java.util.Arrays;

public class ArrayUtil {
  public static double[] concatWithArrayCopy(double[] array1, double[] array2) {
    double[] result = Arrays.copyOf(array1, array1.length + array2.length);
    System.arraycopy(array2, 0, result, array1.length, array2.length);
    return result;
  }
}
