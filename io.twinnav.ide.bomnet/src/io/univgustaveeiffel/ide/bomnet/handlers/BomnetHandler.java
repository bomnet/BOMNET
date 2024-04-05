package io.univgustaveeiffel.ide.bomnet.handlers;

import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspace;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.DirectoryDialog;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.FileDialog;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.ProgressBar;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.e4.core.di.annotations.Execute;
import org.eclipse.e4.ui.services.IServiceConstants;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import org.eclipse.swt.widgets.MessageBox;

import javax.inject.Named; // We use javax instead of jakarta because Omnet++ still uses javax an doesn't have jakarta

public class BomnetHandler {
	
	// ============================================ //
	// ====== EXECUTION SCRIPT OF THE PLUGIN ====== //
	// ============================================ //
    @Execute
    public void execute(@Named(IServiceConstants.ACTIVE_SHELL) Shell s) {
    	
    	// Loading all projects in the workspace
        IWorkspace workspace = ResourcesPlugin.getWorkspace();
        IWorkspaceRoot root = workspace.getRoot();
        
        // Get all the projects in the workspace
        IProject[] projects = root.getProjects();
        
        String versionString = null;
        
        String projectName;
        float inetVersion = 0; // default INET version (INET doesn't exist)
        String inetPath = null;
        
        // Retrieve all INET versions installed in the loaded projects
        for (IProject project : projects) {
            projectName = project.getName();
            
            // To make sure the 'INET' directories would be checked
            projectName = projectName.toLowerCase();
            
            if(projectName.contains("inet")) {
        	    int indexAfterInet = projectName.indexOf("inet") + "inet".length();
        	    versionString = projectName.substring(indexAfterInet).replaceAll("[^0-9.]", "");
        	    
                if (versionString.matches("\\d+(\\.\\d+)?")) { // Is what is after 'inet' numeric in the directory's name
                    try {
                    	// Getting the version as a float
                        float versionFloat = Float.parseFloat(versionString);
                        // If the version of the project is higher than the previous INET version loaded
                        if(inetVersion < versionFloat) {
                        	inetVersion = versionFloat; // The version of the actual project is the higher one
                        	inetPath = project.getLocation().toString(); // Get the path to the version's directory
                        	//MessageDialog.openError(s, "Error", inetPath); DEBUG
                        }
                    } catch (NumberFormatException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        
        // Verify if there are projects in the workspace
        if (projects.length == 0) {
            MessageDialog.openError(s, "Error", "No projects loaded in Eclipse.");
        // Verify if INET is installed
        } else if (inetVersion == 0) {
            MessageDialog.openError(s, "Error", "INET 4.5 or a higher version is required to use this plugin. Please install it.");
        // Verify if INET's version is valid
        } else if(inetVersion < 4.5){
        	MessageDialog.openError(s, "Error", "Your version of INET is obsolete. Please install version 4.5 or higher.");
        // If everything is OK
        } else {
        	
            // Select an IFC file
            FileDialog fileDialog = new FileDialog(s, SWT.OPEN);
            fileDialog.setText("Select IFC file");
            fileDialog.setFilterExtensions(new String[] {"*.ifc"});
            fileDialog.setFilterNames(new String[] {"IFC Files"});
            String selectedFile = fileDialog.open();
            
            // If a file has been selected
            if (selectedFile != null) {
            	
                // Select the directory where the files generated from the IFC file will be saved
            	DirectoryDialog directoryDialog = new DirectoryDialog(s);
            	directoryDialog.setText("Select destination folder for generated files");
            	String selectedFolder = directoryDialog.open();
            	
            	// If a directory has been selected
            	if (selectedFolder != null) {
                    File omnetppIniFile = new File(selectedFolder, "omnetpp.ini");
                    File environmentXmlFile = new File(selectedFolder, "environment.xml");
                    
                    // Check if generated files already exist in the directory
                    if (omnetppIniFile.exists() || environmentXmlFile.exists()) {
                    	MessageBox messageBox = new MessageBox(s, SWT.ICON_WARNING | SWT.YES | SWT.NO);
                        messageBox.setMessage("omnetpp.ini or environment.xml already exists. Do you want to overwrite them?");
                        messageBox.setText("Confirmation");
                        int response = messageBox.open();
                        
                        // If the user confirms, execute the script
                        if (response == SWT.YES) {   
                            executeScript(s, selectedFolder, selectedFile, inetPath);
                            
                        }
                    // If the files do not exist, execute the script directly
                    } else {
                        executeScript(s, selectedFolder, selectedFile, inetPath);
                    }
            	}
            }
        }
    }
    
	// ===================================================== //
	// ====== EXTRACT THE GENERATING SCRIPT AND FILES ====== //
	// ===================================================== //
    private String extractScript(String scriptName) throws IOException {
        // Create a temporary file
        File tempFile = File.createTempFile("script", ".py");
        tempFile.deleteOnExit();

        // Copy the JAR's script to the temporary file
        try (InputStream is = getClass().getResourceAsStream(scriptName);
            OutputStream os = new FileOutputStream(tempFile)) {
            if (is == null) {
                throw new IOException("Script resource not found: " + scriptName);
            }
            byte[] buffer = new byte[1024];
            int bytesRead;
            while ((bytesRead = is.read(buffer)) != -1) {
                os.write(buffer, 0, bytesRead);
            }
        }
        
        // Return absolute path of the extracted script
        return tempFile.getAbsolutePath();
    }
    
    private void extractFiles(String[] listOPathFilesToExtract, String[] listOPathFilesToWrite) throws IOException {
        if (listOPathFilesToExtract.length != listOPathFilesToWrite.length) {
            throw new IllegalArgumentException("Number of paths to extract and write must be equal.");
        }

        for (int i = 0; i < listOPathFilesToExtract.length; i++) {
            String filepathToExtract = listOPathFilesToExtract[i];
            String filepathToWrite = listOPathFilesToWrite[i];

            // Create a temporary file
            File outputFile = new File(filepathToWrite);

            // Copy the JAR's script to the specified file
            try (InputStream is = getClass().getResourceAsStream(filepathToExtract);
                 OutputStream os = new FileOutputStream(outputFile)) {
                if (is == null) {
                    throw new IOException("File resource not found: " + filepathToExtract);
                }
                byte[] buffer = new byte[1024];
                int bytesRead;
                while ((bytesRead = is.read(buffer)) != -1) {
                    os.write(buffer, 0, bytesRead);
                }
            }
        }
    }

	// =========================================== //
	// ====== EXECUTE THE GENERATING SCRIPT ====== //
	// =========================================== //
    private void executeScript(Shell s, String selectedFolder, String selectedFile, String inetPath) {
        Shell dialogShell = new Shell(s, SWT.APPLICATION_MODAL | SWT.DIALOG_TRIM);
        dialogShell.setText("Please wait...");
        dialogShell.setLayout(new GridLayout(1, false));

        Label label = new Label(dialogShell, SWT.NONE);
        label.setText("Extracting IFC data...");

        ProgressBar progressBar = new ProgressBar(dialogShell, SWT.INDETERMINATE);
        progressBar.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, true, false));

        dialogShell.pack();
        dialogShell.open();
        
        Thread scriptExecutionThread = new Thread(() -> {
            StringBuilder output = new StringBuilder();
            int exitCode = 1; // Assume failure by default
            
            try {
                // Extract the Python script in a temporary file on the system
                String scriptPath = extractScript("/io/univgustaveeiffel/ide/bomnet/handlers/generate_files.py");
                
                extractFiles(new String[] {    "/io/univgustaveeiffel/ide/bomnet/handlers/BOMNET_IfcPolyhedron.cc",
                                               "/io/univgustaveeiffel/ide/bomnet/handlers/BOMNET_IfcPolyhedron.h",
                                               "/io/univgustaveeiffel/ide/bomnet/handlers/BOMNET_PhysicalEnvironment.cc",
                                               "/io/univgustaveeiffel/ide/bomnet/handlers/BOMNET_PhysicalEnvironmentCanvasVisualizer.cc",
                                               "/io/univgustaveeiffel/ide/bomnet/handlers/BOMNET_PhysicalEnvironmentOsgVisualizer.cc"},
                             new String[] {    inetPath + "/src/inet/common/geometry/shape/polyhedron/IfcPolyhedron.cc",
                                               inetPath + "/src/inet/common/geometry/shape/polyhedron/IfcPolyhedron.h",
                                               inetPath + "/src/inet/environment/common/PhysicalEnvironment.cc",
                                               inetPath + "/src/inet/visualizer/canvas/environment/PhysicalEnvironmentCanvasVisualizer.cc",
                                               inetPath + "/src/inet/visualizer/osg/environment/PhysicalEnvironmentOsgVisualizer.cc"});
                
                // Determine the command based on the OS
                String osName = System.getProperty("os.name").toLowerCase();
                String command = osName.contains("nix") || osName.contains("nux") || osName.contains("aix") || osName.contains("mac") 
                                 ? "python3 " : "python ";
                command += scriptPath + " " + selectedFolder + " " + selectedFile + " " + inetPath;
                
                // Execute the command
                ProcessBuilder pb = new ProcessBuilder(command.split(" "));
                pb.redirectErrorStream(true); // Merge standard and error streams
                Process p = pb.start();
                
                // Capture the script's output
                BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
                String line;
                while ((line = in.readLine()) != null) {
                    output.append(line).append("\n");
                }
                in.close();
                
                // Capture the exit code
                exitCode = p.waitFor();
            } catch (Exception e) {
                output.append("Exception occurred: ").append(e.getMessage()).append("\n");
            }
            if (exitCode == 0) {
	            Display.getDefault().asyncExec(() -> {
	            	dialogShell.close();
	                MessageBox messageBox;
	                // Execution was successful
	                messageBox = new MessageBox(s, SWT.ICON_INFORMATION | SWT.OK);
	                messageBox.setText("Simulation Files Produced");
	                messageBox.setMessage("Simulation files have been successfully produced in " + selectedFolder);
	
	                messageBox.open();
	                refreshWorkspace();
	                });
            } else {
                Display.getDefault().asyncExec(() -> {
                	dialogShell.close();
                    MessageBox messageBox;
                    // There was an error during execution
                    output.append("\nProcess exited with code: ").append(1); // Display the captured output in a dialog
                    messageBox = new MessageBox(s, SWT.ICON_ERROR | SWT.OK);
                    messageBox.setText("Script Execution Output");
                    messageBox.setMessage(output.toString());
                    messageBox.open();
                    refreshWorkspace();
                	});
                }
        });
        scriptExecutionThread.start();
    }

	// =================================== //
	// ====== REFRESH THE WORKSPACE ====== //
	// =================================== //
    private void refreshWorkspace() {
        IWorkspace workspace = ResourcesPlugin.getWorkspace();
        try {
            // Refresh the workspace to see the update files
            workspace.getRoot().refreshLocal(IResource.DEPTH_INFINITE, null);
        } catch (CoreException e) {
            e.printStackTrace();
        }
    }
}
