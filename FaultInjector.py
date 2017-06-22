#!/usr/bin/python
import dronekit_sitl
# Import DroneKit-Python
from dronekit import connect, VehicleMode
from Tkinter import *

# Connect to the Vehicle.
#print("Connecting to vehicle on: %s" % (connection_string,))
def connectToDrone(address):
  vehicle = connect('127.0.0.1:14551', wait_ready=True)

  # Get some vehicle attributes (state)
  print "Get some vehicle attribute values:"
  print " GPS: %s" % vehicle.gps_0
  print " Battery: %s" % vehicle.battery
  print " Last Heartbeat: %s" % vehicle.last_heartbeat
  print " Is Armable?: %s" % vehicle.is_armable
  print " System status: %s" % vehicle.system_status.state
  print " Mode: %s" % vehicle.mode.name    # settable

  print "Changing mode to circle"
  vehicle.mode = VehicleMode("CIRCLE");
  print "Mode changed to Circle"

  # Close vehicle object before exiting script
  vehicle.close()

  # Shut down simulator
  #sitl.stop()
  print("Completed")

def work():
  print("called work")
  
def main():
  root = Tk()
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
  portBox.insert(0, "14551")  
  portBox.pack(side=LEFT, padx=2, pady=2)
  
  b = Button(toolbar, text="Connect", width=6, command=connect)
  b.pack(side=LEFT, padx=2, pady=2)  

  toolbar.pack(side=TOP, fill=X)
  
  root.mainloop()

if __name__ == "__main__":
  main()
