#!/usr/bin/python
import dronekit_sitl
# Import DroneKit-Python
from dronekit import connect, VehicleMode
from Tkinter import *
import time
import thread
# Connect to the Vehicle.
#print("Connecting to vehicle on: %s" % (connection_string,))
updatePanes = None
vehicle = None
root = None

#connects to a drone sitting at ip:port and dispatches a thread to display it's
#inforamtion to the readout window
def connectToDrone(ip, port):
  print "Connecting To Drone"
  global vehicle
  #connect to vehicle at ip:port
  vehicle = connect(ip+':'+port, wait_ready=True)
  # create thread to update readout information in real time
  thread.start_new_thread(updateVehicleStatus, (vehicle,))

#disconnects the vehicle and cleans the readout
def disconnect():
  #close vehicle connection
  vehicle.close()
  updateReadoutWindow(updatePanes[0], "Disconneced")

#continually updates the readout with vehicle information
def updateVehicleStatus(vehicle):
  while(1): 
    #update the readout with vehicle information
    updateReadoutWindow(updatePanes[0], "%s" % vehicle.gps_0 +
    "\n%s" % vehicle.battery +
    "\nLast Heartbeat: %s" % vehicle.last_heartbeat +
    "\nIs Armable?: %s" % vehicle.is_armable +
    "\nSystem status: %s" % vehicle.system_status.state +
    "\nMode: %s" % vehicle.mode.name)  
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
  
  #creates IP label
  ipLabel = Label(toolbar, text="IP Address")
  ipLabel.pack(side=LEFT, padx=2, pady=2)
  
  #creates IP entry box
  ipBox = Entry(toolbar)
  ipBox.delete(0, END)
  ipBox.insert(0, "127.0.0.1")  
  ipBox.pack(side=LEFT, padx=2, pady=2)
 
  #creates port label
  portLabel = Label(toolbar, text="Port")
  portLabel.pack(side=LEFT, padx=2, pady=2)

  #creates port entry box
  portBox = Entry(toolbar)
  portBox.delete(0, END)
  portBox.insert(0, "14550")  
  portBox.pack(side=LEFT, padx=2, pady=2)
  
  #creates connection button
  con = Button(toolbar, text="Connect", width=6, command=lambda: connectToDrone(ipBox.get(), portBox.get()))
  con.pack(side=LEFT, padx=2, pady=2)  
  #creates disconnect button
  dis = Button(toolbar, text="Disconnect", width=6, command=disconnect)
  dis.pack(side=LEFT, padx=2, pady=2)
  toolbar.pack(side=TOP, fill=X)

#creates a split panned window, with a text box on the left, and fault buttons on the left
def loadInfoPane(root):
  m = PanedWindow(orient=HORIZONTAL)
  m.pack(fill=BOTH, expand=1) 
  
  #creates a text box on the left side (this is the readout window)
  text2 = Text(root, height=20, width=50)
  text2.pack(side=LEFT)
  text2.config(state=DISABLED)
  
  m.add(text2)

  #creates a simple paned window on the right side
  T2 = PanedWindow(orient=VERTICAL)
  m.add(T2)
  
  #returns panes
  return [text2, T2]

def wind():
  print "boy, it's windy"

#adds faults to the window
def createFaultButtons(pane):
  #add wind button
  windB = Button(pane, text="Wind", width = 3, command = wind)
  windB.pack()

def main():
  global root
  root = Tk()
  root.geometry("800x600")
  loadToolbar(root)
  global updatePanes 
  updatePanes = loadInfoPane(root)
  updateReadoutWindow(updatePanes[0],"Use the connect button to connect to a drone!")
  createFaultButtons(updatePanes[1])
  root.mainloop()
  
if __name__ == "__main__":
  main()
