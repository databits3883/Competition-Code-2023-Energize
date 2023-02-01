package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Filesystem;

public class RobotSourceCodeInformation {

    private static NetworkTable table;
    private String branchString;
    private String commitString;
    private String dateString;
    private StringPublisher branchPub;
    private StringPublisher commitPub;
    private StringPublisher datePub;
  
    public RobotSourceCodeInformation() {
      table = NetworkTableInstance.getDefault().getTable("source-code-information");
    }
  
    public void init() {
      File deployDir = Filesystem.getDeployDirectory();
      File branchFile = new File(deployDir, "branch.txt");
      File commitFile = new File(deployDir, "commit.txt");
      File dateFile = new File(deployDir, "date.txt");

      try {
        BufferedReader branchReader = new BufferedReader(new FileReader(branchFile));
        branchString = branchReader.readLine();
        branchReader.close();

        BufferedReader commitReader = new BufferedReader(new FileReader(commitFile));
        commitString = commitReader.readLine();
        commitReader.close();

        BufferedReader dateReader = new BufferedReader(new FileReader(dateFile));
        dateString = dateReader.readLine();
        dateReader.close();
        
        System.out.println(branchString);
        System.out.println(commitString);
        System.out.println(dateString);

        branchPub = table.getStringTopic("branch-string").publish();
        commitPub = table.getStringTopic("commit-string").publish();
        datePub = table.getStringTopic("date-string").publish();

        branchPub.set(branchString);
        commitPub.set(commitString);
        datePub.set(dateString);
      }
      catch (Exception e)
      {
        System.out.println(e);
      }
    }
  
    public void close() {
      branchPub.close();
      commitPub.close();
    }
  }