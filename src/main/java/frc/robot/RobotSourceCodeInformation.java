package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Filesystem;

public class RobotSourceCodeInformation {

    private static NetworkTable table;
    private String branchString;
    private String commitString;
    private String dateString;
    private String workingDirStatusString;
    private String buildDateString;

    private StringPublisher branchPub;
    private StringPublisher commitPub;
    private StringPublisher datePub;
    private StringPublisher workingDirStatusPub;
    private StringPublisher buildDatePub;
  
    public RobotSourceCodeInformation() {
      table = NetworkTableInstance.getDefault().getTable("source-code-information");
    }
  
    public void init() {
      File deployDir = Filesystem.getDeployDirectory();
      File branchFile = new File(deployDir, "branch.txt");
      File commitFile = new File(deployDir, "commit.txt");
      File dateFile = new File(deployDir, "date.txt");
      File workingDirStatusFile = new File(deployDir, "status.txt");
      File buildDateFile = new File(deployDir, "build.txt");

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
        
        BufferedReader workingDirStatusReader = new BufferedReader(new FileReader(workingDirStatusFile));
        workingDirStatusString = workingDirStatusReader.readLine();
        workingDirStatusReader.close();

        BufferedReader buildDateReader = new BufferedReader(new FileReader(buildDateFile));
        buildDateString = buildDateReader.readLine();
        buildDateReader.close();

        System.out.println(branchString);
        System.out.println(commitString);
        System.out.println(dateString);
        System.out.println(workingDirStatusString);
        System.out.println(buildDateString);

        branchPub = table.getStringTopic("commit-branch").publish();
        commitPub = table.getStringTopic("commit-sha").publish();
        datePub = table.getStringTopic("commit-date").publish();
        workingDirStatusPub = table.getStringTopic("working-dir-status").publish();
        buildDatePub = table.getStringTopic("build-date").publish();

        branchPub.set(branchString);
        commitPub.set(commitString);
        datePub.set(dateString);
        workingDirStatusPub.set(workingDirStatusString);
        buildDatePub.set(buildDateString);
      }
      catch (Exception e)
      {
        System.out.println(e);
      }
    }
  
    public void close() {
      branchPub.close();
      commitPub.close();
      datePub.close();
      workingDirStatusPub.close();
      buildDatePub.close();
    }
  }