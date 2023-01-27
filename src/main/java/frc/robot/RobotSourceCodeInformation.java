package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.Filesystem;

public class RobotSourceCodeInformation {

    private static NetworkTable table;
    private String branchString;
    private String commitString;
    private StringPublisher branchPub;
    private StringPublisher commitPub;
  
    public RobotSourceCodeInformation() {
      table = NetworkTableInstance.getDefault().getTable("source-code-information");
    }
  
    public void init() {
      File deployDir = Filesystem.getDeployDirectory();
      File branchFile = new File(deployDir, "branch.txt");
      File commitFile = new File(deployDir, "commit.txt");
      
      try {
        BufferedReader branchReader = new BufferedReader(new FileReader(branchFile));
        branchString = branchReader.readLine();
        branchReader.close();

        BufferedReader commitReader = new BufferedReader(new FileReader(commitFile));
        branchString = commitReader.readLine();
        commitReader.close();

        branchPub = table.getStringTopic("branch-string").publish();
        commitPub = table.getStringTopic("commit-string").publish();

        branchPub.set(branchString);
        commitPub.set(commitString);
      }
      catch (IOException e)
      {
        System.out.println(e);
      }
    }
  
    public void close() {
      branchPub.close();
      commitPub.close();
    }
  }