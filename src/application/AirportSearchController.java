package application;

import java.text.NumberFormat;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import application.data.Data;
import application.model.Airport;
import application.model.AirportFlights;
import application.model.Flight;
import application.model.FlightsHistory;
import javafx.beans.property.SimpleStringProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.collections.ObservableMap;
import javafx.fxml.FXML;
import javafx.scene.chart.CategoryAxis;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Label;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.stage.Stage;

public class AirportSearchController {

	@FXML
	private Label errormsg;
	
	@FXML
	private Label airline1name;
	
	@FXML
	private Label airline2name;	

	@FXML
	private Label airline3name;
	
	@FXML
	private Label airline4name;	
	
	@FXML
	private Label airline5name;	

	@FXML
	private Label airline1flights;
	
	@FXML
	private Label airline2flights;	

	@FXML
	private Label airline3flights;
	
	@FXML
	private Label airline4flights;	
	
	@FXML
	private Label airline5flights;
	
	@FXML
	private Label averagearrivaldelay;
	
	@FXML
	private Label averagedeparturedelay;	
	
	@FXML
	private Label delay10mins;	
	
	@FXML
	private Label delay10to30mins;	
	
	@FXML
	private Label delaymorethan30mins;
	
	@FXML
	private Label general1;	
	
	@FXML
	private Label general2;
	
	@FXML
	private Label title;
	
    @FXML
    private TableView arrivalsTable;
    
    @FXML
    private TableView departuresTable;
    
    @FXML
    private CategoryAxis airlineNameCategory = new CategoryAxis();
    
    private NumberAxis yAxis = new NumberAxis();
   
    
    @FXML
    private LineChart<String, Number> delayChart = new LineChart<String,Number>(airlineNameCategory,yAxis);
    
    
	
    private Stage dialogStage;
	private String searchBar;
	private Data flightData;
	private ObservableList<AirportChart> arrivalData = FXCollections.observableArrayList();
	private ObservableList<AirportChart> departureData = FXCollections.observableArrayList();
	
	public AirportSearchController(){
		flightData = Data.getInstance();
	}
	
	
	@FXML
    private void initialize() {
    }
	
	
	 public void setDialogStage(Stage dialogStage) {
	        this.dialogStage = dialogStage;
	 }


	public void saveSearch(String input) {
		searchBar = input;
	}


	public void updateData() {
	    List<Airport> airportList = flightData.getAirportList();
		AirportFlights flights;
		
		Map<String, Integer> topAirlines = new TreeMap<String,Integer>();
		
		int nationalFlights = 0;
		int internationalFlights = 0;
		
		int delayCounter1 = 0;
		int delayCounter2 = 0;
		int delayCounter3 = 0;
		
		for (int i = 0; i < airportList.size(); i++) {
				flights = flightData.getAirportFlights(airportList.get(i));
				Airport curAirport = flights.getAirport();
				String airportCode = curAirport.getCode();
				String airportName = curAirport.getName();
				
				FlightsHistory departures = flights.getDepartures();
				FlightsHistory arrivals = flights.getArrivals();
				
				ObservableMap<String, Flight> departureFlights = departures.getFlights();
				ObservableMap<String, Flight> arrivalFlights = arrivals.getFlights();
				
			    if (airportName.toUpperCase().equals(searchBar.toUpperCase()) || airportCode.equals(searchBar.toUpperCase())) {
					//Retrieve all the data.
					
					title.setText(airportName + " - " + airportCode);
					averagearrivaldelay.setText((int)(arrivals.getDelay()) + " minutes.");
					averagedeparturedelay.setText((int)(departures.getDelay()) + " minutes.");
					
					nationalFlights += flights.getNumNationalFlights();
					internationalFlights += flights.getNumInternationalFlights();
				    
					Iterator<Map.Entry<String, Flight>> entries = departureFlights.entrySet().iterator();
					while (entries.hasNext()) {
					    Map.Entry<String, Flight> entry = entries.next();
					    Flight curFlight = entry.getValue();
					    String flightCompany = curFlight.getCompany();
					    Double delay = curFlight.getDelay();
					    
					    departureData.add(new AirportChart(curFlight.getFlightNumber(), curFlight.getCompany(), "No origin", curFlight.getDestiny(), curFlight.getDateTime().toString(), (int)(curFlight.getDelay()) + "", curFlight.getTerminal()));
					    
					    if (delay <= 10.0) {
					    	delayCounter1++;
					    } else if (delay >= 10.0 && delay <= 30.0) {
					    	delayCounter2++;
					    } else if (delay >= 30.0) {
					    	delayCounter3++;
					    }
					    
					    if (topAirlines.containsKey(flightCompany)) {
							topAirlines.put(flightCompany, topAirlines.get(flightCompany) + 1);
						} else if (!topAirlines.containsKey(flightCompany)) {
							topAirlines.put(flightCompany, 1);
						}
					    
					}
					Iterator<Map.Entry<String, Flight>> entries2 = arrivalFlights.entrySet().iterator();
					while (entries2.hasNext()) {
					    Map.Entry<String, Flight> entry = entries2.next();
					    Flight curFlight = entry.getValue();
					    String flightCompany = curFlight.getCompany();
					    Double delay = curFlight.getDelay();
					    
					    arrivalData.add(new AirportChart(curFlight.getFlightNumber(), curFlight.getCompany(), curFlight.getOrigin(), "No destination", curFlight.getDateTime().toString(), (int)(curFlight.getDelay()) + "", curFlight.getTerminal()));
					    
					    if (delay <= 10.0) {
					    	delayCounter1++;
					    } else if (delay >= 10.0 && delay <= 30.0) {
					    	delayCounter2++;
					    } else if (delay >= 30.0) {
					    	delayCounter3++;
					    }
					    
					    if (topAirlines.containsKey(flightCompany)) {
							topAirlines.put(flightCompany, topAirlines.get(flightCompany) + 1);
						} else if (!topAirlines.containsKey(flightCompany)) {
							topAirlines.put(flightCompany, 1);
						}
					    
					}
					
					
				}
		 }
		general1.setText(NumberFormat.getInstance().format(nationalFlights) + "");
		general2.setText(NumberFormat.getInstance().format(internationalFlights) + "");
		
		//Generates Delay Information
		delay10mins.setText(NumberFormat.getInstance().format(delayCounter1) + "");
		delay10to30mins.setText(NumberFormat.getInstance().format(delayCounter2) + "");
		delaymorethan30mins.setText(NumberFormat.getInstance().format(delayCounter3) + "");
		
		//Generates Top Airlines
		Map<Integer, String> topAirlinesSwapped = new TreeMap<Integer,String>();
		topAirlinesSwapped = reverse(topAirlines);
		
		Map<Integer, String> topAirlinesOrdered = new TreeMap<Integer, String>(Collections.reverseOrder());
		topAirlinesOrdered.putAll(topAirlinesSwapped);	

		//Makes sure iteration is in order.
		Map<Integer, String> topAirlinesOrderedFinal = new LinkedHashMap<Integer, String>();
		topAirlinesOrderedFinal.putAll(topAirlinesOrdered);				
		
		XYChart.Series series = new XYChart.Series();
		Iterator<Map.Entry<Integer, String>> entries2 = topAirlinesOrderedFinal.entrySet().iterator();
		int count2 = 1;
		while (entries2.hasNext()) {
		    Map.Entry<Integer, String> entry = entries2.next();
			if (count2 == 1) {
				airline1flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline1name.setText(entry.getValue());
			} else if (count2 == 2) {
				airline2flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline2name.setText(entry.getValue());
			} else if (count2 == 3) {
				airline3flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline3name.setText(entry.getValue());
			} else if (count2 == 4) {
				airline4flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline4name.setText(entry.getValue());				
			} else if (count2 == 5) {
				airline5flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline5name.setText(entry.getValue());
			}
			series.getData().add(new XYChart.Data(entry.getValue(), entry.getKey()));
			count2++;
			if(count2 == 9) {
				break;
			}
			
			//Line Chart setup
			delayChart.setTitle("Top Airlines - Comparison");
			delayChart.setLegendVisible(false);
			delayChart.getData().add(series);
			series = new XYChart.Series();

			
			//Generates tables for Airport information   
	        TableColumn flightNumber = new TableColumn("Flight Number");
	        flightNumber.setMinWidth(100);
	        flightNumber.setCellValueFactory(new PropertyValueFactory<>("flightNumber")); 

	        TableColumn airline = new TableColumn("Airline");
	        airline.setMinWidth(100);
	        airline.setCellValueFactory(new PropertyValueFactory<>("airline")); 
	        
	        TableColumn origin = new TableColumn("Origin");
	        origin.setMinWidth(100);
	        origin.setCellValueFactory(new PropertyValueFactory<>("origin")); 
	        
	        TableColumn destination = new TableColumn("Destination");
	        destination.setMinWidth(100);
	        destination.setCellValueFactory(new PropertyValueFactory<>("destination")); 
	        
	        TableColumn dateTime = new TableColumn("Date/Time");
	        dateTime.setMinWidth(100);
	        dateTime.setCellValueFactory(new PropertyValueFactory<>("dateTime")); 
	        
	        TableColumn delay = new TableColumn("Delay (in mins)");
	        delay.setMinWidth(100);
	        delay.setCellValueFactory(new PropertyValueFactory<>("delay")); 
	        
	        TableColumn terminal = new TableColumn("Terminal");
	        terminal.setMinWidth(100);
	        terminal.setCellValueFactory(new PropertyValueFactory<>("terminal")); 
	        
	        arrivalsTable.setItems(arrivalData);
	        departuresTable.setItems(departureData);
	        
	        arrivalsTable.getColumns().addAll(flightNumber, airline, origin, dateTime, delay, terminal);
	        departuresTable.getColumns().addAll(flightNumber, airline, destination, dateTime, delay, terminal);

	       
		}
		
		
		
	}
	//Swaps keys and values of a hashmap.
	public static <K,V> HashMap<V,K> reverse(Map<K,V> map) {
	    HashMap<V,K> rev = new HashMap<V, K>();
	    for(Map.Entry<K,V> entry : map.entrySet())
	        rev.put(entry.getValue(), entry.getKey());
	    return rev;
	}
	 
    public static class AirportChart {
        private SimpleStringProperty flightNumber;
        private SimpleStringProperty airline;
        private SimpleStringProperty origin;
        private SimpleStringProperty destination;
        private SimpleStringProperty dateTime;
        private SimpleStringProperty delay;
        private SimpleStringProperty terminal;

        private AirportChart(String flightNumber, String airline, String origin, String destination, String dateTime, String delay, String terminal) {
            this.flightNumber = new SimpleStringProperty(flightNumber);
            this.airline = new SimpleStringProperty(airline);
            this.origin = new SimpleStringProperty(origin);
            this.destination = new SimpleStringProperty(destination);
            this.dateTime = new SimpleStringProperty(dateTime);
            this.delay = new SimpleStringProperty(delay);
            this.terminal = new SimpleStringProperty(terminal);
        }

        public String getFlightNumber() {
            return flightNumber.get();
        }
        public String getAirline() {
            return airline.get();
        }
        public String getOrigin() {
            return origin.get();
        }
        public String getDestination() {
            return destination.get();
        }
        public String getDateTime() {
            return dateTime.get();
        }
        public String getDelay() {
            return delay.get();
        }
        public String getTerminal() {
            return terminal.get();
        }
       
    }
	 
	 
}
