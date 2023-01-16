package frc.robot.util;

import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;

public class FileManager {
    
    public File file;
    PrintWriter writer;
    Scanner reader;

    public FileManager(String name){
        file = new File(name);
    }

    public boolean exists(){
        return file.exists();
    }

    public String readLine(){
        if(reader == null){
            try{
                reader = new Scanner(file);
            } catch(Exception e){
                e.printStackTrace();
            }
        }

        if(reader.hasNextLine()){
            return reader.nextLine();
        }else{
            return null;
        }
    }

    public void writeLine(String line){
        if(writer == null){
            try{
                writer = new PrintWriter(file);
            } catch(Exception e){
                e.printStackTrace();
            }
        }

        writer.println(line);
    }

    public void close(){
        if(reader != null){
            reader.close();
        }
        if(writer != null){
            writer.close();
        }
    }
}
