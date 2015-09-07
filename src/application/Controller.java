package application;

import java.text.DateFormatSymbols;
import java.text.NumberFormat;
import java.util.Calendar;
import java.util.Collections;
import java.util.GregorianCalendar;
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
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.chart.BarChart;
import javafx.scene.chart.CategoryAxis;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.PieChart;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Label;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.control.cell.PropertyValueFactory;

public class Controller {

	@FXML
	private Label general1;
	
	@FXML
	private Label general2;
	
	@FXML
	private Label busiest1name;
	
	@FXML
	private Label busiest2name;	

	@FXML
	private Label busiest3name;
	
	@FXML
	private Label busiest4name;	
	
	@FXML
	private Label busiest5name;	
	
	@FXML
	private Label busiest6name;	
	
	@FXML
	private Label busiest7name;	
	
	@FXML
	private Label busiest8name;	

	@FXML
	private Label busiest1flights;
	
	@FXML
	private Label busiest2flights;	

	@FXML
	private Label busiest3flights;
	
	@FXML
	private Label busiest4flights;	
	
	@FXML
	private Label busiest5flights;	
	
	@FXML
	private Label busiest6flights;	
	
	@FXML
	private Label busiest7flights;	
	
	@FXML
	private Label busiest8flights;
	
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
	private Label airline6name;	
	
	@FXML
	private Label airline7name;	
	
	@FXML
	private Label airline8name;	

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
	private Label airline6flights;	
	
	@FXML
	private Label airline7flights;	
	
	@FXML
	private Label airline8flights;
	
	@FXML
	private Label plane1name;
	
	@FXML
	private Label plane2name;	

	@FXML
	private Label plane3name;
	
	@FXML
	private Label plane4name;	
	
	@FXML
	private Label plane5name;	
	
	@FXML
	private Label plane6name;	
	
	@FXML
	private Label plane7name;	
	
	@FXML
	private Label plane8name;
	
	@FXML
	private Label plane1flights;
	
	@FXML
	private Label plane2flights;	

	@FXML
	private Label plane3flights;
	
	@FXML
	private Label plane4flights;	
	
	@FXML
	private Label plane5flights;	
	
	@FXML
	private Label plane6flights;	
	
	@FXML
	private Label plane7flights;	
	
	@FXML
	private Label plane8flights;
	
	@FXML
	private Label delay10mins;	
	
	@FXML
	private Label delay10to30mins;	
	
	@FXML
	private Label delaymorethan30mins;
	
	@FXML
	private Label dateTime;
	
	@FXML
	private Label dateTime1;
	
    @FXML
    private CategoryAxis xAxis = new CategoryAxis();
    
    @FXML
    private NumberAxis yAxis = new NumberAxis();
    
    @FXML
    private BarChart<String, Number> topAirplaneTypes = new BarChart<String, Number>(xAxis, yAxis);
    
    @FXML
    private PieChart topAirlines;
    
    @FXML
    private TextField searchBar;
    
    @FXML
    private Label errorMsg;
    
    @FXML
    private TableView listofAirports;
    
	private Data flightData;
	private ObservableList<Airports> data = FXCollections.observableArrayList();
	private ObservableList<PieChart.Data> pieChartData = FXCollections.observableArrayList();
	
	
	public Controller(){
		flightData = Data.getInstance();
	}
    
	@FXML
    private void initialize() throws Exception {

		
		//Loads Global Information
		List<Airport> airportList = flightData.getAirportList();
		AirportFlights flights;
		errorMsg.setText("");
		
		//Generates date
		createDate();
		
		//General Data
		int nationalFlights = 0;
		int internationalFlights = 0;
		Map<Integer, String> busiestAirports = new TreeMap<Integer,String>();
		Map<String, Integer> topAirlines = new TreeMap<String,Integer>();
		Map<String, Integer> topAirplaneTypes = new TreeMap<String,Integer>();
		
		
	
		int delayCounter1 = 0;
		int delayCounter2 = 0;
		int delayCounter3 = 0;
		
		for (int i = 0; i < airportList.size(); i++) {
			flights = flightData.getAirportFlights(airportList.get(i));
			
			Airport curAirport = flights.getAirport();
			data.add(new Airports(curAirport.getName(), curAirport.getCode()));
			
			nationalFlights += flights.getNumNationalFlights();
			internationalFlights += flights.getNumInternationalFlights();
			
			busiestAirports.put(flights.getNumFlights(), flights.getAirport().getCode());
			
			FlightsHistory departures = flights.getDepartures();
			FlightsHistory arrivals = flights.getArrivals();
			
			ObservableMap<String, Flight> departureFlights = departures.getFlights();
			ObservableMap<String, Flight> arrivalFlights = arrivals.getFlights();
			
			Iterator<Map.Entry<String, Flight>> entries = departureFlights.entrySet().iterator();
			while (entries.hasNext()) {
			    Map.Entry<String, Flight> entry = entries.next();
			    Flight curFlight = entry.getValue();
			    String flightCompany = curFlight.getCompany();
			    String flightPlaneType = curFlight.getPlane();
			    Double delay = curFlight.getDelay();
			    
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

		    	if (flightPlaneType != null) {
				    if (topAirplaneTypes.containsKey(flightPlaneType)) {
				    	topAirplaneTypes.put(flightPlaneType, topAirplaneTypes.get(flightPlaneType) + 1);
					} else if (!topAirplaneTypes.containsKey(flightPlaneType)) {
						topAirplaneTypes.put(flightPlaneType, 1);
					}		    		
		    	}

			    
			}
			Iterator<Map.Entry<String, Flight>> entries2 = arrivalFlights.entrySet().iterator();
			while (entries2.hasNext()) {
			    Map.Entry<String, Flight> entry = entries2.next();
			    Flight curFlight = entry.getValue();
			    String flightCompany = curFlight.getCompany();
			    String flightPlaneType = curFlight.getPlane();
			    
			    if (topAirlines.containsKey(flightCompany)) {
					topAirlines.put(flightCompany, topAirlines.get(flightCompany) + 1);
				} else if (!topAirlines.containsKey(flightCompany)) {
					topAirlines.put(flightCompany, 1);
				}
			    
		    	if (flightPlaneType != null) {
				    if (topAirplaneTypes.containsKey(flightPlaneType)) {
				    	topAirplaneTypes.put(flightPlaneType, topAirplaneTypes.get(flightPlaneType) + 1);
					} else if (!topAirplaneTypes.containsKey(flightPlaneType)) {
						topAirplaneTypes.put(flightPlaneType, 1);
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
		
		//Generates Busiest Airports
		Map<Integer, String> busiestAirportsOrdered = new TreeMap<Integer, String>(Collections.reverseOrder());
		busiestAirportsOrdered.putAll(busiestAirports);
		
		//Makes sure iteration is in order.
		Map<Integer, String> busiestAirportsOrderedFinal = new LinkedHashMap<Integer, String>();
		busiestAirportsOrderedFinal.putAll(busiestAirportsOrdered);
		
		Iterator<Map.Entry<Integer, String>> entries = busiestAirportsOrderedFinal.entrySet().iterator();
		int count = 1;
		while (entries.hasNext()) {
		    Map.Entry<Integer, String> entry = entries.next();
		    //System.out.println("Key = " + entry.getKey() + ", Value = " + entry.getValue());
			if (count == 1) {
				busiest1flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest1name.setText(entry.getValue());
			} else if (count == 2) {
				busiest2flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest2name.setText(entry.getValue());
			} else if (count == 3) {
				busiest3flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest3name.setText(entry.getValue());
			} else if (count == 4) {
				busiest4flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest4name.setText(entry.getValue());				
			} else if (count == 5) {
				busiest5flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest5name.setText(entry.getValue());						
			} else if (count == 6) {
				busiest6flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest6name.setText(entry.getValue());						
			} else if (count == 7) {
				busiest7flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest7name.setText(entry.getValue());						
			} else if (count == 8) {
				busiest8flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				busiest8name.setText(entry.getValue());						
			}			
			count++;
			if(count == 9) {
				break;
			}
		}
		
		//Generates Top Airlines
		Map<Integer, String> topAirlinesSwapped = new TreeMap<Integer,String>();
		topAirlinesSwapped = reverse(topAirlines);
		
		Map<Integer, String> topAirlinesOrdered = new TreeMap<Integer, String>(Collections.reverseOrder());
		topAirlinesOrdered.putAll(topAirlinesSwapped);	

		//Makes sure iteration is in order.
		Map<Integer, String> topAirlinesOrderedFinal = new LinkedHashMap<Integer, String>();
		topAirlinesOrderedFinal.putAll(topAirlinesOrdered);				
		
		Iterator<Map.Entry<Integer, String>> entries2 = topAirlinesOrderedFinal.entrySet().iterator();
		int count2 = 1;
		while (entries2.hasNext()) {
		    Map.Entry<Integer, String> entry = entries2.next();
		    //System.out.println("Key = " + entry.getKey() + ", Value = " + entry.getValue());
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
			} else if (count2 == 6) {
				airline6flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline6name.setText(entry.getValue());						
			} else if (count2 == 7) {
				airline7flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline7name.setText(entry.getValue());						
			} else if (count2 == 8) {
				airline8flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				airline8name.setText(entry.getValue());						
			}			
			pieChartData.add(new PieChart.Data(entry.getValue() + " (" + NumberFormat.getInstance().format(entry.getKey()) + ")", entry.getKey()));
			count2++;
			if(count2 == 9) {
				break;
			}
		}		
		
		//Generates Top Plane Types
		Map<Integer, String> topAirplaneTypesSwapped = new TreeMap<Integer,String>();
		topAirplaneTypesSwapped = reverse(topAirplaneTypes);
		
		Map<Integer, String> topAirplaneTypesOrdered = new TreeMap<Integer, String>(Collections.reverseOrder());
		topAirplaneTypesOrdered.putAll(topAirplaneTypesSwapped);	

		//Makes sure iteration is in order.
		Map<Integer, String> topAirplaneTypesOrderedFinal = new LinkedHashMap<Integer, String>();
		topAirplaneTypesOrderedFinal.putAll(topAirplaneTypesOrdered);				
		
		XYChart.Series series1 = new XYChart.Series();
		Iterator<Map.Entry<Integer, String>> entries3 = topAirplaneTypesOrderedFinal.entrySet().iterator();
		int count3 = 1;
		while (entries3.hasNext()) {
		    Map.Entry<Integer, String> entry = entries3.next();
		    //System.out.println("Key = " + entry.getKey() + ", Value = " + entry.getValue());
			if (count3 == 1) {
				plane1flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane1name.setText(entry.getValue());
			} else if (count3 == 2) {
				plane2flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane2name.setText(entry.getValue());
			} else if (count3 == 3) {
				plane3flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane3name.setText(entry.getValue());
			} else if (count3 == 4) {
				plane4flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane4name.setText(entry.getValue());				
			} else if (count3 == 5) {
				plane5flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane5name.setText(entry.getValue());						
			} else if (count3 == 6) {
				plane6flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane6name.setText(entry.getValue());						
			} else if (count3 == 7) {
				plane7flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane7name.setText(entry.getValue());						
			} else if (count3 == 8) {
				plane8flights.setText(NumberFormat.getInstance().format(entry.getKey()) + "");
				plane8name.setText(entry.getValue());						
			}		
	        series1.getData().add(new XYChart.Data(entry.getValue(), entry.getKey()));
			count3++;
			if(count3 == 9) {
				break;
			}
		}
		
		//Generates Chart Data
        
		//Bar Chart setup
		xAxis.setLabel("Plane Type");       
        yAxis.setLabel("Flights");

        this.topAirplaneTypes.getData().addAll(series1);
        
        //Pie Chart setup
        this.topAirlines.setTitle("Top Airlines");
        this.topAirlines.getData().addAll(pieChartData);

		
		//Generates List of Airports   
        TableColumn airportNameCol = new TableColumn("Airport Name");
        airportNameCol.setMinWidth(400);
        airportNameCol.setCellValueFactory(new PropertyValueFactory<>("airportName")); 

        TableColumn airportCodeCol = new TableColumn("Airport Code");
        airportCodeCol.setMinWidth(200);
        airportCodeCol.setCellValueFactory(new PropertyValueFactory<>("airportCode"));
        
        listofAirports.setItems(data);
        listofAirports.getColumns().addAll(airportNameCol, airportCodeCol);
				
    }
	
	  public static Object getKeyFromValue(Map hm, Object value) {
		    for (Object o : hm.keySet()) {
		      if (hm.get(o).equals(value)) {
		        return o;
		      }
		    }
		    return null;
		  }
	
	//Swaps keys and values of a hashmap.
	public static <K,V> HashMap<V,K> reverse(Map<K,V> map) {
	    HashMap<V,K> rev = new HashMap<V, K>();
	    for(Map.Entry<K,V> entry : map.entrySet())
	        rev.put(entry.getValue(), entry.getKey());
	    return rev;
	}
	
	private void createDate() {
    	Calendar cal = new GregorianCalendar();
    	int month = cal.get(Calendar.MONTH);
    	int day = cal.get(Calendar.DAY_OF_MONTH);
    	int year = cal.get(Calendar.YEAR);    		    	
		dateTime.setText(getMonthForInt(month)+" "+day+", "+year);
		dateTime1.setText(getMonthForInt(month)+" "+day+", "+year);
	}
	
	//Gets month name for date.
    private String getMonthForInt(int num) {
        String month = "wrong";
        DateFormatSymbols dfs = new DateFormatSymbols();
        String[] months = dfs.getMonths();
        if (num >= 0 && num <= 11 ) {
            month = months[num];
        }
        return month;
    }

	
	 @FXML
	 private void handleSearchAirport(ActionEvent event) {
		 if (searchBar.getText().equals("")) {
			errorMsg.setText("ERROR: No input.");
		 } else {
		     List<Airport> airportList = flightData.getAirportList();
			 AirportFlights flights;
			 
			 boolean isMatch = false;
			 for (int i = 0; i < airportList.size(); i++) {
					flights = flightData.getAirportFlights(airportList.get(i));
					Airport curAirport = flights.getAirport();
					String airportCode = curAirport.getCode();
					String airportName = curAirport.getName();
					
					if (airportName.toUpperCase().equals(searchBar.getText().toUpperCase()) || airportCode.equals(searchBar.getText().toUpperCase())) {
						isMatch = true;
						break;
					}
			 }
			 
			 if (isMatch) {
				 String searchInput = searchBar.getText();
				 errorMsg.setText("");
				 Main.showSearchResults(searchInput);				 
			 } else {
				 errorMsg.setText("ERROR: Invalid Airport Code."); 
			 }
			 

		 }
		 

		   
		}
	 
	    public static class Airports {
	        private final SimpleStringProperty airportName;
	        private final SimpleStringProperty airportCode;

	        private Airports(String aName, String lCode) {
	            this.airportName = new SimpleStringProperty(aName);
	            this.airportCode = new SimpleStringProperty(lCode);
	        }

	        public String getAirportName() {
	            return airportName.get();
	        }
	        public String getAirportCode() {
	            return airportCode.get();
	        }
	        public String toString() {
	        	return "Airport name: " + airportName + ", airport code: " + airportCode;
	        }
	       
	       
	    }
	   
		 
	 }



