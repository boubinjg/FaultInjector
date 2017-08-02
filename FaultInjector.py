#!/usr/bin/python
#from dronekit_sitl import SITL
# Import DroneKit-Python
from dronekit import connect, VehicleMode
from Tkinter import *
import time, thread, sys, struct, os
from curses import ascii
from pymavlink import mavparm, mavutil
from pymavlink.dialects.v10 import common as mavlink

# globals
#TK panes which need to be updated
updatePanes = None
#dronekit vehicle
vehicle = None
#TK window root 
root = None
#bool which denotes whether you are connected to SITL/Mavproxy or not
connected = False

#toggle buttons:
gpsButton = None
rcButton = None
battButton = None
thrButton = None
GCSButton = None

#Button Activity
gcsfs = bfs = tfs = rfs = gpsfs = "Inactive"

#connects to a drone sitting at ip:port and dispatches a thread to display it's
#inforamtion to the readout window
def connectToDrone(dkip, dkport):

   
  #Dronekit connection
  global vehicle
  #connect to vehicle at ip:port
  print("Attempting To Connect to Dronekit")
  vehicle = connect(dkip+':'+dkport, wait_ready=True)
  #set connected bool to True
  global connected 
  connected = True
  
  #initialize globals associated with the vehicle
  #Throttle PWM fail safe value
  global THR_FS_VAL
  THR_FS_VAL = vehicle.parameters['THR_FS_VALUE']
  #Battery capacity failsafe value
  global FS_BATT_MAH
  FS_BATT_MAH = vehicle.parameters['FS_BATT_MAH'] 
  #ID of the ground control station
  global SYSID_MYGCS
  SYSID_MYGCS = vehicle.parameters['SYSID_MYGCS']

  # create thread to update readout information in real time
  thread.start_new_thread(updateVehicleStatus, (vehicle,))
  
#disconnects the vehicle and cleans the readout
def disconnect():
  #close vehicle connection
  vehicle.close()
  updateReadoutWindow(updatePanes[0], "Disconneced")
  global connected
  connected = False

#continually updates the readout with vehicle information
def updateVehicleStatus(vehicle):
  while(connected): 
    #update the readout with vehicle information  
    updateText = '';
    updateText += ("System Status: %s" % vehicle.system_status.state + 
		      "\nLast Heartbeat: %s" %vehicle.last_heartbeat + 
		      "\nMode: %s" % vehicle.mode.name +
		      "\nIs Atmable?: %s" % vehicle.is_armable + "\n")
    #Add battery/location/environment info
    updateText += ("\nBattery Capacity: %s MAH" % vehicle.parameters['BATT_CAPACITY'] +
                   "\nGPS Info: %s" % vehicle.gps_0 + 
	 	   "\nLattitude: %s " % vehicle.location.global_relative_frame.lat +
		   "\nLongitude: %s" % vehicle.location.global_relative_frame.lon +
                   "\nAirspeed: %s" % vehicle.velocity +
                   "\nAltitude: %s" % vehicle.location.global_relative_frame.alt +
		   "\nWind Speed: %s" % vehicle.parameters['SIM_WIND_SPD'] +
		   "\nWind Direction: %s\n" % vehicle.parameters['SIM_WIND_DIR'])
    #add failsafes
    updateText += ("\nGPS Failsafe:      " + gpsfs +
		   "\nRadio Failsafe:    " + rfs +
		   "\nThrottle Failsafe: "  + tfs +
		   "\nBattery Failsafe:  " + bfs +
		   "\nGCS Failsafe:      " + gcsfs) 

    #update the readout
    updateReadoutWindow(updatePanes[0], updateText)
    #update root
    root.update()
    #wait for 1 second
    time.sleep(1)

#helper function to write information to the readout
def updateReadoutWindow(textWindow, text):
  #sets the readout window from read only to read/write
  textWindow.config(state=NORMAL)
  #clears the readout window
  textWindow.delete('1.0', END)
  #inserts text into readout window
  textWindow.insert(END, text)
  #sets window back to read only 
  textWindow.config(state=DISABLED)

#adds toolbar to root frame
def loadToolbar(root):
  #Creates toolbar frame
  toolbar = Frame(root);
  mpToolbar = Frame(toolbar);
  sToolbar = Frame(toolbar);

  mpLabel = Label(mpToolbar, text = "Connect to Drone: ")
  mpLabel.pack(side=LEFT, padx=2, pady=2)
  #creates IP label
  MPipLabel = Label(mpToolbar, text="IP Address")
  MPipLabel.pack(side=LEFT, padx=2, pady=2)
  
  #creates IP entry box
  MPipBox = Entry(mpToolbar)
  MPipBox.delete(0, END)
  MPipBox.insert(0, "127.0.0.1")  
  MPipBox.pack(side=LEFT, padx=2, pady=2)
 
  #creates port label
  MPportLabel = Label(mpToolbar, text="Port")
  MPportLabel.pack(side=LEFT, padx=2, pady=2)

  #creates port entry box
  MPportBox = Entry(mpToolbar)
  MPportBox.delete(0, END)
  MPportBox.insert(0, "14551")  
  MPportBox.pack(side=LEFT, padx=2, pady=2)
  
  #creates connection button
  MPcon = Button(mpToolbar, text="Connect", width=6, command=lambda: connectToDrone(MPipBox.get(), MPportBox.get()))
  MPcon.pack(side=LEFT, padx=2, pady=2)  
  #creates disconnect button
  MPdis = Button(mpToolbar, text="Disconnect", width=6, command=disconnect)
  MPdis.pack(side=LEFT, padx=2, pady=2)
  
  #creates connection button
  mpToolbar.pack(side=TOP, fill=X)
  sToolbar.pack(side=TOP, fill=X)
  toolbar.pack(side=TOP, fill=X)
  
#creates a split panned window, with a text box on the left, and fault buttons on the left
def loadInfoPane(root):
  window = PanedWindow(orient=HORIZONTAL)
  window.pack(fill=BOTH, expand=1) 
  
  leftSubwindow = PanedWindow(orient=VERTICAL) 
  #TBH not sure what bottom left will do yet, but it's here
  bottomLeft = PanedWindow(orient=HORIZONTAL)
  bottomLeft.pack()

  #creates a text box on the left side (this is the readout window)
  readout = Text(root, height=20, width=50)
  readout.pack(side=LEFT)
  readout.config(state=DISABLED)
  
  leftSubwindow.add(readout)
  leftSubwindow.add(bottomLeft)
  
  window.add(leftSubwindow)

  #creates a simple paned window on the right side
  buttonArray = PanedWindow(orient=VERTICAL)
  window.add(buttonArray)
  
  #returns panes
  return [readout, buttonArray, bottomLeft]

'''The following 6 functions are called when their corresponding Fault Buttons are pressed.
   Each one communicates with SITL and Dronekit to read and set variables inside the vehicle
   and simulation'''
def wind(windSPD, windDIR):
  #create mavproxy parameter dictionary
  mav_param = mavparm.MAVParmDict()
  #set mavproxy parameters over sitl
  vehicle.parameters['SIM_WIND_DIR'] = float(windDIR)
  vehicle.parameters['SIM_WIND_SPD'] = float(windSPD)

def gps():
  #get global button and failsafe readout text
  global gpsButton, gpsfs
  #create param dictionary
  mav_param = mavparm.MAVParmDict()
  # if button's text is to Disable GPS
  if gpsButton.configure('text')[-1] == 'Disable GPS':
        #set SITL to disable gps
        vehicle.parameters['SIM_GPS_DISABLE'] = float(1)
	#change button text
	gpsButton.configure(text='Enable GPS')
	#change readout text
	gpsfs = "Active"
  #if text is Enable gps
  else:
	#set SITL to enable gps
  	vehicle.parameters['SIM_GPS_DISABLE'] = float(0) 
	#change readout text
	gpsfs = "Inactive"
        #change button text
	gpsButton.configure(text='Disable GPS')

def rc():
  #get global button and failsafe readout text
  global rcButton, rfs
  #create param dictionary
  mav_param = mavparm.MAVParmDict()
  #if button text says Disable RC
  if rcButton.configure('text')[-1] == 'Disable RC':
	#Send param to disable RC
	vehicle.parameters['SIM_RC_FAIL'] = float(1)
	#change readout text
	rfs = "Active"
	#change button text
	rcButton.configure(text='Enable RC')
	
  else:
	#reactivate RC
  	vehicle.parameters['SIM_RC_FAIL'] = float(0)
	#change readout text
	rfs = "Inactive"
	#change button text
	rcButton.configure(text='Disable RC')
	

def throttle():
  #create parameter dictionary
  mav_param = mavparm.MAVParmDict()
  #get global dronekit vehiclea, button, current throttle PWM failsafe value, and failsafe readout text
  global vehicle, thrButton, THR_FS_VAL, tfs
  #if button says to Activate Throttle Failsafe
  if thrButton.configure('text')[-1] == 'Activate Throttle Failsafe':
	#set throttle failsafe
        vehicle.parameters['THR_FS_VALUE'] = float(2000)
	#change readout
	tfs = "Active"
	#change button text
	thrButton.configure(text="Deactivate Throttle Failsafe")
  else:
	#Set Throttle pwn failsafe value back to normal
	vehicle.parameters['THR_FS_VALUE'] = float(THR_FS_VAL)
	#change button text
	thrButton.configure(text="Activate Throttle Failsafe")
	#change readout text
	tfs = "Inactive"
	
def battery():
  #create param dictionary
  mav_param = mavparm.MAVParmDict()
  #get global dronekit vehicle, button, battert MAH,and failsafe readout text
  global vehicle, battButton, FS_BATT_MAH, bfs
  #if button text says to activate battery failsafe
  if battButton.configure('text')[-1] == 'Activate Battery Failsafe':
	#set battery FS value above current capacity
  	vehicle.parameters['FS_BATT_MAH'] = float(4000)
	#set readout text
	bfs = "Active"
	#set button text
	battButton.configure(text="Deactivate Battery Failsafe")
  else:
	#set battery capacity back to normal
	vehicle.parameters['FS_BATT_MAH'] = float(FS_BATT_MAH)
	#set readout
	bfs = "Inactive"
	#set button text
	battButton.configure(text="Activate Battery Failsafe")

def gcs():
  #create param dict
  mav_param = mavparm.MAVParmDict()
  #get global dronekit vehicle, button, gcs ID and failsafe readout text
  global vehicle, GCSButton, SYSID_MYGCS, gcsfs
  #if button says Disconnect GCS
  if GCSButton.configure('text')[-1] == 'Disconnect GCS':
	#set GCS ID to something unrecognizable to the vehicle
        vehicle.parameters['SYSID_MYGCS'] = float(0)
	#change button text
	GCSButton.configure(text="Reconnect GCS")
	#change readout
	gcsfs = "Active"
  else:
	#set GCS id back
   	vehicle.parameters['SYSID_MYGCS'] = float(SYSID_MYGCS)	
        #change button text
  	GCSButton.configure(text="Disconnect GCS")
	#change readout
  	gcsfs = "Inactive"
  
#adds faults to the window
def createFaultButtons(pane):
  #add wind button
  windPane = Frame(pane)
  #create wind speed frame and labels
  windSPDFrame = Frame(windPane)
  mpLabel = Label(windSPDFrame, text = "Set Wind Speed: ")
  mpLabel.pack(side=TOP, padx=2, pady=2)
  #add wind speed input box
  windSPDBox = Entry(windSPDFrame)
  windSPDBox.delete(0, END)
  windSPDBox.insert(0, "0")  
  windSPDBox.pack(side=TOP, padx=2, pady=2)
  
  windSPDFrame.pack(side=TOP)
  #create wind direction frame and label
  windDIRFrame = Frame(windPane)
 
  mpLabel = Label(windDIRFrame, text = "Set Wind Direction in Degrees: ")
  mpLabel.pack(side=TOP, padx=2, pady=2)
  #add wind direction input box
  windDIRBox = Entry(windDIRFrame)
  windDIRBox.delete(0, END)
  windDIRBox.insert(0, "0")  
  windDIRBox.pack(side=TOP, padx=2, pady=2)

  windDIRFrame.pack(side=TOP)
  #add wind button
  windB = Button(windPane, text="Set Wind", width = 8,  command=lambda: wind(windSPDBox.get(), windDIRBox.get()))
  windB.pack(pady=5)
  windPane.pack()
  #add gps Frame and button
  gpsPane = Frame(pane)
  global gpsButton
  gpsButton = Button(gpsPane, text = "Disable GPS", width = 8, command=lambda: gps())
  gpsButton.pack(pady=5);
  gpsPane.pack();
  #add rc Frame and button
  rcPane = Frame(pane)
  global rcButton
  rcButton = Button(rcPane, text = "Disable RC", width = 8, command=lambda: rc())
  rcButton.pack(pady=5);
  rcPane.pack();
  #add throttle frame and button
  thrPane = Frame(pane)
  global thrButton
  thrButton = Button(thrPane, text = "Activate Throttle Failsafe", width = 20, command=lambda: throttle())
  thrButton.pack(pady=5);
  thrPane.pack();
  
  #add battery frame and button
  battPane = Frame(pane)
  global battButton
  battButton = Button(battPane, text = "Activate Battery Failsafe", width = 20, command=lambda: battery())
  battButton.pack(pady=5)
  battPane.pack();
  #add gps frame and button
  GCSPane = Frame(pane)
  global GCSButton
  GCSButton = Button(GCSPane, text = "Disconnect GCS", width = 20, command=lambda: gcs())
  GCSButton.pack(pady=5);
  GCSPane.pack(); 

def main():
  global root
  #create root window
  root = Tk()
  #add window title
  root.title("Fault Injector")
  #set window size
  root.geometry("760x420")
  #add the connections toolbar onto the root window
  loadToolbar(root)
  global updatePanes 
  #create the panes for the readout window and the fault buttons
  updatePanes = loadInfoPane(root)
  #update the readout window
  updateReadoutWindow(updatePanes[0],"Use the connect button to connect to a drone!")
  #add fault buttons to root window
  createFaultButtons(updatePanes[1])
  #loop 
  root.mainloop()
  
if __name__ == "__main__":
  main()
