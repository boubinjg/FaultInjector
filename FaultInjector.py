#!/usr/bin/python
import dronekit_sitl
# Import DroneKit-Python
from dronekit import connect, VehicleMode
from Tkinter import *

# Connect to the Vehicle.
#print("Connecting to vehicle on: %s" % (connection_string,))
updatePanes = None
vehicle = None
root = None
def connectToDrone(ip, port):
  print "Connecting To Drone"
  #vehicle = connect('127.0.0.1:14551', wait_ready=True)
  global vehicle
  vehicle = connect(ip+':'+port, wait_ready=True)
  # Get some vehicle attributes (state)
  
  print "Get some vehicle attribute values:"
  print " GPS: %s" % vehicle.gps_0
  print " Battery: %s" % vehicle.battery
  print " Last Heartbeat: %s" % vehicle.last_heartbeat
  print " Is Armable?: %s" % vehicle.is_armable
  print " System status: %s" % vehicle.system_status.state
  print " Mode: %s" % vehicle.mode.name    # settable
  updateVehicleStatus(vehicle)
  # Close vehicle object before exiting script
  #vehicle.close()

  # Shut down simulator
  #sitl.stop()
  print("Completed")

def updateVehicleStatus(vehicle):
  while(1): 
    updatePanes[0]["text"] = ("Get some vehicle attribute values:\n"+ " GPS: %s" % vehicle.gps_0 +
    " Battery: %s" % vehicle.battery +
    " Last Heartbeat: %s" % vehicle.last_heartbeat +
    " Is Armable?: %s" % vehicle.is_armable +
    " System status: %s" % vehicle.system_status.state +
    " Mode: %s" % vehicle.mode.name)   # settable
    root.update()

def loadToolbar(root):
  toolbar = Frame(root);
  
  ipLabel = Label(toolbar, text="IP Address")
  ipLabel.pack(side=LEFT, padx=2, pady=2)
  
  ipBox = Entry(toolbar)
  ipBox.delete(0, END)
  ipBox.insert(0, "127.0.0.1")  
  ipBox.pack(side=LEFT, padx=2, pady=2)
 
  portLabel = Label(toolbar, text="Port")
  portLabel.pack(side=LEFT, padx=2, pady=2)

  portBox = Entry(toolbar)
  portBox.delete(0, END)
  portBox.insert(0, "14550")  
  portBox.pack(side=LEFT, padx=2, pady=2)
  
  b = Button(toolbar, text="Connect", width=6, command=lambda: connectToDrone(ipBox.get(), portBox.get()))
  b.pack(side=LEFT, padx=2, pady=2)  

  toolbar.pack(side=TOP, fill=X)

def loadInfoPane(root):
  m = PanedWindow(orient=HORIZONTAL)
  m.pack(fill=BOTH, expand=1) 
  
  T = Text(m, height=1, width=40)
  T.grid_propagate(False)
  T.config(state=DISABLED) 
  m.add(T);
  
  T2 = PanedWindow(orient=VERTICAL)
  m.add(T2)
  
  return [m]

def main():
  global root
  root = Tk()
  root.geometry("800x600")
  loadToolbar(root)
  global updatePanes 
  updatePanes = loadInfoPane(root)
  root.mainloop()
  
if __name__ == "__main__":
  main()
