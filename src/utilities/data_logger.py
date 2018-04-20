#!/usr/bin/env python3
import pickle
from time import strftime
from os import getcwd
from os.path import join, split
from csv import writer
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from tkinter import Tk, E, SUNKEN, W, ttk, LEFT, TOP, filedialog, END, RIGHT,StringVar, BOTH
from networktables import NetworkTables
from networktables.instance import NetworkTablesInstance
import pandas as pd


class DataLoggerConfigData():
    """
    The Data Logger Configuration Data class.
    """

    def __init__(self):
        self.address = StringVar()
        self.path = StringVar()
        self.file_name = StringVar()
        try:
            pf = open(join(getcwd(), ".dlc_config"), "rb")
            dlc_config = pickle.load(pf)
            pf.close()
            self.address.set(dlc_config["address"])
            self.path.set(dlc_config["path"])
            self.file_name.set(dlc_config["file_name"])
        except:
            self.address.set("10.46.7.2")
            self.path.set(r"C:\Users\CIS 4607\logs")
            self.file_name.set("")

    def onClosing(self):
        pf = open(join(getcwd(), ".dlc_config"), "wb")
        pickle.dump({"path": dlcd.path.get(),
                     "address": dlcd.address.get(),
                     "file_name": dlcd.file_name.get()}, pf)
        pf.close()
        root.destroy()


class DataLoggerClient():
    """
    The Data Logger Client class.
    """

    def __init__(self, root, config_data):
        self.dl = None
        self.lf = None
        self.dl_keys = []
        self.root = root
        self.cd = config_data

        # Create the tkinter widgets
        self.server_address_label = ttk.Label(self.root, text="Server Address")
        self.server_address_entry = ttk.Entry(self.root, width=60,
                                              textvariable=self.cd.address)
        self.log_path_label = ttk.Label(self.root, text="Log File Path")
        self.log_path_entry = ttk.Entry(self.root, width=60,
                                        textvariable=self.cd.path)
        self.make_connection_button = ttk.Button(self.root, text="Connect",
                                                 command=self.makeConnection)
        self.logging_button = ttk.Button(self.root, text="Start Logging",
                                         command=self.toggleLoggingButton)
        self.status = ttk.Label(self.root,
                                text="Data Logger Client initialized...",
                                relief=SUNKEN, anchor=E)

        # Layout all of the tkinter widgets
        self.server_address_label.grid(padx=5, pady=5, row=0, column=0)
        self.server_address_entry.grid(padx=5, pady=5, row=0, column=1,
                                       columnspan=2)
        self.log_path_label.grid(padx=5, row=1, column=0)
        self.log_path_entry.grid(padx=5, row=1, column=1, columnspan=2)
        self.make_connection_button.grid(padx=5, pady=5, row=2, column=1)
        self.logging_button.grid(padx=5, pady=5, row=2, column=2)
        self.status.grid(row=3, column=0, columnspan=3, sticky=(E, W))

    def makeConnection(self):
        NetworkTablesInstance.initialize(NetworkTables,
                                         self.server_address_entry.get())
        self.dl = NetworkTablesInstance.getTable(NetworkTables, "SmartDashboard")
        self.status.config(text="Connected to the SmartDashboard network table...")

    def toggleLoggingButton(self):
        if self.logging_button.config("text")[-1] == "Start Logging":
            if self.dl is None:
                self.status.config(text="Click \"Connect\" before trying to "
                                        "log data!!!")
            elif not NetworkTablesInstance.isConnected(NetworkTables):
                    self.status.config(text="Not connected to the Network"
                                            " Tables!!!")
            else:
                self.cd.file_name.set(join(self.log_path_entry.get(),
                                           strftime("%Y%m%d-%H%M%S") + ".txt"))
                try:
                    self.lf = open(self.cd.file_name.get(), "w", newline='')
                    self.lf_csv_writer = writer(self.lf, delimiter=',')
                except IOError:
                    self.status.config(text="Failed to open %s!!!" %
                                       (self.cd.file_name.get()))
                self.dl.addEntryListener(listener=self.timeStampChanged,
                                         key="TimeStamp")
                self.dl_keys = self.dl.getKeys()
                self.lf_csv_writer.writerow(self.dl_keys)
                self.status.config(text="Started logging data...")
                self.logging_button.config(text="Stop Logging")
        else:
            self.dl.removeEntryListener(listener=self.timeStampChanged)
            self.lf.close()
            self.status.config(text="Stopped logging data...")
            self.logging_button.config(text="Start Logging")

    def timeStampChanged(self, table, key, value, isNew):
        row = []
        for key in self.dl_keys:
            row.append(self.dl.getEntry(key).getDouble(0.0))
        self.lf_csv_writer.writerow(row)
        self.lf.flush()


class DataLoggerPlotter():
    """
    The Data Logger Plotter class.
    """

    def __init__(self, root, config_data):
        self.root = root
        self.cd = config_data
        self.data_table = None

        # Create the tkinter widgets
        self.top_frame = ttk.Frame(self.root)
        self.left_frame = ttk.Frame(self.root)
        self.right_frame = ttk.Frame(self.root)
        self.log_label = ttk.Label(self.top_frame, text="Input Log File")
        self.log_entry = ttk.Entry(self.top_frame, width=60,
                                   textvariable=self.cd.file_name)
        self.new_button = ttk.Button(self.top_frame, text="New",
                                     command=self.loadNewFile)
        self.x_label = ttk.Label(self.right_frame, text="X")
        self.x_var = StringVar()
        self.x_var.set("Select X")
        self.x_combobox = ttk.Combobox(self.right_frame, width=45,
                                       textvariable=self.x_var)
        self.y_label = ttk.Label(self.right_frame, text="Y")
        self.y_var = StringVar()
        self.y_var.set("Select Y")
        self.y_combobox = ttk.Combobox(self.right_frame, width=45,
                                       textvariable=self.y_var)
        self.plot_button = ttk.Button(self.right_frame, text="Plot Data",
                                      command=self.plotData)

        self.figure = Figure(figsize=(8, 4.5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.left_frame)
        self.canvas.show()
        self.toolbar = NavigationToolbar2TkAgg(self.canvas, self.left_frame)

        # Layout all of the tkinter widgets
        self.top_frame.pack(side=TOP)
        self.left_frame.pack(side=LEFT, fill=BOTH)
        self.right_frame.pack(side=RIGHT, fill=BOTH)
        self.log_label.pack(side=LEFT, pady=5)
        self.log_entry.pack(side=LEFT, pady=5, padx=5)
        self.new_button.pack(side=LEFT, pady=5)
        self.canvas.get_tk_widget().pack(side=TOP)
        self.canvas._tkcanvas.pack(side=TOP)
        self.x_label.pack(side=TOP)
        self.x_combobox.pack(side=TOP)
        self.y_label.pack(side=TOP)
        self.y_combobox.pack(side=TOP)
        self.plot_button.pack(side=TOP)

    def loadNewFile(self):
        [path, file_name] = split(filedialog.askopenfilename(
            initialdir=self.cd.path.get(), title="Select file"))
        self.cd.path.set(path)
        self.cd.file_name.set(join(path, file_name))

    def createDataTable(self):
        try:
            self.data_table = pd.read_csv(self.cd.file_name.get())
            header_list = list(self.data_table)
            self.x_combobox['values'] = header_list
            self.y_combobox['values'] = header_list
        except:
            pass

    def plotData(self):
        x = self.data_table[self.x_combobox.get()].values
        y = self.data_table[self.y_combobox.get()].values
        if self.ax.lines:
            self.line.set_ydata(y)
        else:
            self.line, = self.ax.plot(x, y, marker='x')
        self.ax.set_ylim([min(y), max(y)])
        self.ax.set_xlim([min(x), max(x)])
        self.canvas.draw()
        self.toolbar.update()

    def onVisibility(self, event):
        self.createDataTable()


matplotlib.use('TkAgg')
root = Tk()
root.title("Data Logger Client")
tabControl = ttk.Notebook(root)
logging_tab = ttk.Frame(tabControl)
tabControl.add(logging_tab, text="Logging")
plotting_tab = ttk.Frame(tabControl)
tabControl.add(plotting_tab, text="Plotting")
tabControl.pack(expand=1, fill="both")
dlcd = DataLoggerConfigData()
dlc = DataLoggerClient(logging_tab, dlcd)
dlp = DataLoggerPlotter(plotting_tab, dlcd)
plotting_tab.bind("<Visibility>", dlp.onVisibility)
root.protocol("WM_DELETE_WINDOW", dlcd.onClosing)
root.resizable(1, 1)
root.mainloop()
