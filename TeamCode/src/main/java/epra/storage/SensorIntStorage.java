package epra.storage;

public class SensorIntStorage {
    private int[] sensorValues = new int[0];
    /**Stores values from sensors in integer form.
     *<p></p>
     *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     *<p></p>
     *Intended to store values from motor encoders. */
    public SensorIntStorage(int[] startValues) {setSensorValues(startValues);}

    /**Sets the stored values to match an array.*/
    public void setSensorValues(int[] setValues) {
        if (sensorValues.length != setValues.length) {sensorValues = new int[setValues.length];}
        for (int ii = 0; ii < setValues.length; ii++) {sensorValues[ii] = setValues[ii];}
    }
    /**Sets the value of a certain int in the stored array. If provided index is out of range, will return false.*/
    public boolean setSensorValue(int value, int index) {
        if (index < sensorValues.length) {sensorValues[index] = value;
    return true;} else {return false;}
    }
    /**Returns the stored int array.*/
    public int[] getSensorValues() {return sensorValues;}
    /**Returns a certain value from the stored array if successful. If provided index is out of range, will return 0.*/
    public int getSensorValue(int index) {if (index < sensorValues.length) {return sensorValues[index];}
        else {return 0;}
    }
}
