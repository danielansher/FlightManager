/*
 * Daniel Ansher
 * May 25, 2015
 * 
 * Flight Dashboard program that handles Spanish flight data from AENA airports.
 */
package application;
	
import java.io.IOException;
import java.net.URL;

import application.data.Data;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.stage.Modality;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;


public class Main extends Application {
	
	private static Stage primaryStage;
	private static URL airportSearchURL;
	
	@Override
	public void start(Stage primaryStage) {
		this.primaryStage = primaryStage;
		this.primaryStage.setTitle("AENA Flight Information Dashboard");
		this.primaryStage.getIcons().add(new Image("view/favicon.png"));
		try {
	        Pane root = (Pane)FXMLLoader.load(getClass().getResource("/view/DashboardOverview.fxml"));		
	        
	        //Need to store URL for AirportSearch GUI.
	        airportSearchURL = getClass().getResource("/view/AirportSearch.fxml");
	        
	        Scene scene = new Scene(root,800,618);
			scene.getStylesheets().add(getClass().getResource("application.css").toExternalForm());
		
			
			primaryStage.setScene(scene);
			primaryStage.show();
		} catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	public static boolean showSearchResults(String input) {
	    try {
	        // Load the fxml file and create a new stage for the popup dialog.
	        FXMLLoader loader = new FXMLLoader();
	        
	        loader.setLocation(airportSearchURL);

	        AnchorPane page = (AnchorPane) loader.load();

	        // Create the dialog Stage.
	        Stage dialogStage = new Stage();
	        dialogStage.setTitle("Airport Statistics");
	        dialogStage.initModality(Modality.WINDOW_MODAL);
	        dialogStage.initOwner(primaryStage);
	        Scene scene = new Scene(page);
	        dialogStage.setScene(scene);

	        // Set the Receta into the controller.
	        AirportSearchController controller = loader.getController();
	        controller.setDialogStage(dialogStage);
	        controller.saveSearch(input);
	        controller.updateData();


	        // Show the dialog and wait until the user closes it
	        dialogStage.showAndWait();
	        
	        
	        return true;
	    } catch (IOException e) {
	        e.printStackTrace();
	        return false;
	    }
	      
	}

	public static void main(String[] args) {
		launch(args);
	}
}
