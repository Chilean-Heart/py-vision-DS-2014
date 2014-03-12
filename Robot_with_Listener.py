import time
from pynetworktables import *

SmartDashboard.init()

table = NetworkTable.GetTable("datatable")

class Listener(ITableListener):
    def __init__(self):
        ITableListener.__init__(self)

    def ValueChanged(self, table, key, value, isNew):
        print('Value changed: key %s, isNew: %s: %s' % (key, isNew, table.GetValue(key)))

listener = Listener()

table.AddTableListener(listener)

ready = False
connection = False
connectString = 'l'
redAlliance = True

while True:
    if not ready:
        try:
            table.PutBoolean("connection", connection)
            ready = True
        except TableKeyNotDefinedException:
            time.sleep(0.1)

    connectString = raw_input("Connect or change alliance? Y/N; R/B: ")
    if connectString == 'y' or connectString == 'Y':
        connection = True
        ready = False
    elif connectString == 'n' or connectString == 'N':
        connection = False
        ready = False
    elif connectString == 'r' or connectString == 'R':
        redAlliance = True
    elif connectString == 'b' or connectString == 'B':
        redAlliance = False

    try:
        table.PutBoolean("allianceRed", redAlliance)
    except TableKeyNotDefinedException:
        time.sleep(0.1)
        
    time.sleep(0.01)
