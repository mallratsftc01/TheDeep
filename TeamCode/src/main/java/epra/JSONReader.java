package epra;

import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.Step;

import java.io.FileReader;

/**A class to read json files for auto.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class JSONReader {

    private static Gson gson = new Gson();

    /**Reads auto steps from a json.
     * @param fileName The filepath of the the json.
     * @return An array of auto steps.*/
    public static Step[] readSteps(String fileName) {
        Step[] directions = {};
        try (FileReader reader = new FileReader(fileName)) {
            directions = gson.fromJson(reader, Step[].class);
        } catch (Exception e) {}
        return directions;
    }

    /**Reads filepaths from a json.
     * @param fileName The filepath of the json.
     * @return An array of filepaths.*/
    public static String[] readAuto(String fileName) {
        String[] files = {};
        try (FileReader reader = new FileReader(fileName)) {
            files = gson.fromJson(reader, String[].class);
        } catch (Exception e) {}
        return files;
    }
}
