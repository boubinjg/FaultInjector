#!/usr/bin/python
import dronekit_sitl
# Import DroneKit-Python
from dronekit import connect, VehicleMode
import tkinter as tk

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

def __init__(self, master):	
  self.master = master
  self.frame = tk.Frame(self.master)
  self.button1 = tk.Button(self.frame, text = 'New Window', width = 25, command = self.new_window)
  self.button1.pack()
  self.frame.pack()

def main():
  root = tk.Tk()
  root.mainloop()

if __name__ == "__main__":
  main()
