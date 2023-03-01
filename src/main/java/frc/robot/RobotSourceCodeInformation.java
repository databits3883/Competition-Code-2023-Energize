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
    private String messageString;
    private String dateString;
    private String workingDirStatusString;
    private String buildDateString;

    private StringPublisher branchPub;
    private StringPublisher commitPub;
    private StringPublisher messagePub;
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
      File messageFile = new File(deployDir, "message.txt");
      File dateFile = new File(deployDir, "date.txt");
      File workingDirStatusFile = new File(deployDir, "status.txt");
      File buildDateFile = new File(deployDir, "build.txt");

      try {
        BufferedReader branchReader = new BufferedReader(new FileReader(branchFile));
        branchString = branchReader.readLine();
        branchReader.close();
        System.out.println(branchString);
        branchPub = table.getStringTopic("commit-branch").publish();
        branchPub.set(branchString);
      }
      catch (Exception e)
      {
        System.out.println(e);
      }

      try {
        BufferedReader commitReader = new BufferedReader(new FileReader(commitFile));
        commitString = commitReader.readLine();
        commitReader.close();
        System.out.println(commitString);
        commitPub = table.getStringTopic("commit-sha").publish();
        commitPub.set(commitString);

        BufferedReader messageReader = new BufferedReader(new FileReader(messageFile));
        messageString = messageReader.readLine();
        messageReader.close();
        System.out.print(messageString);
        messagePub = table.getStringTopic("commit-message").publish();
        messagePub.set(messageString);
      }
      catch (Exception e)
      {
        System.out.println(e);
      }

      try {
        BufferedReader dateReader = new BufferedReader(new FileReader(dateFile));
        dateString = dateReader.readLine();
        dateReader.close();
        System.out.println(dateString);
        datePub = table.getStringTopic("commit-date").publish();
        datePub.set(dateString);
      }
      catch (Exception e)
      {
        System.out.println(e);
      }

      try {
        BufferedReader workingDirStatusReader = new BufferedReader(new FileReader(workingDirStatusFile));
        workingDirStatusString = workingDirStatusReader.readLine();
        workingDirStatusReader.close();
        System.out.println(workingDirStatusString);
        workingDirStatusPub = table.getStringTopic("working-dir-status").publish();
        workingDirStatusPub.set(workingDirStatusString);
      }
      catch (Exception e)
      {
        System.out.println(e);
      }

      try {
        BufferedReader buildDateReader = new BufferedReader(new FileReader(buildDateFile));
        buildDateString = buildDateReader.readLine();
        buildDateReader.close();
        System.out.println(buildDateString);
        buildDatePub = table.getStringTopic("build-date").publish();
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
      messagePub.close();
      datePub.close();
      workingDirStatusPub.close();
      buildDatePub.close();
    }
  }