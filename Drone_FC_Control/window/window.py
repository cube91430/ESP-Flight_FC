import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout,
    QHBoxLayout, QComboBox, QTextEdit, QLineEdit, QMessageBox,
    QFrame, QStackedLayout, QTableWidget, QTableWidgetItem, 
    QDialog, QRadioButton, QButtonGroup, QLineEdit, QGridLayout
)
from PyQt5.QtCore import QTimer
from datetime import datetime

class SerialApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OekoFlight")
        self.serial_port = serial.Serial()
        self.init_ui()

    def init_ui(self):
        # === HEADER BAR ===
        port_label = QLabel("Port:")
        self.port_box = QComboBox()
        self.refresh_ports()

        auto_search_btn = QPushButton("Auto Search")
        auto_search_btn.clicked.connect(self.refresh_ports)

        baudrate_label = QLabel("Baudrate:")
        self.baudrate_box = QComboBox()
        self.baudrate_box.addItems(["9600", "115200", "57600", "38400", "19200", "14400", "4800", "2400", "1200"])

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)

        header_layout = QHBoxLayout()
        header_layout.addWidget(port_label)
        header_layout.addWidget(self.port_box)
        header_layout.addWidget(auto_search_btn)
        header_layout.addWidget(baudrate_label)
        header_layout.addWidget(self.baudrate_box)
        header_layout.addWidget(self.connect_btn)
        header_layout.addStretch()

        # === LEFT SIDEBAR ===
        self.sidebar = QVBoxLayout()
        self.sidebar.setSpacing(5)

        self.page_names = [
            "Setup", "Configuration", "Power and Battery", "Presets",
            "Modes", "Motors", "Interface", "Blackbox", "CLI", "Mapping"
        ]
        self.buttons = []
        for index, name in enumerate(self.page_names):
            btn = QPushButton(name)
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda _, idx=index: self.page_layout.setCurrentIndex(idx))
            self.sidebar.addWidget(btn)
            self.buttons.append(btn)
        self.sidebar.addStretch()

        # === PAGE STACK (Main Content Area) ===
        self.page_layout = QStackedLayout()
        for index, name in enumerate(self.page_names):
            page = QWidget()
            layout = QVBoxLayout()
            
            if index == 0:  # Setup page
                self.output_display = QTextEdit()
                self.output_display.setReadOnly(True)

                self.input_line = QLineEdit()
                self.send_btn = QPushButton("Send")
                self.send_btn.clicked.connect(self.send_data)

                input_layout = QHBoxLayout()
                input_layout.addWidget(self.input_line)
                input_layout.addWidget(self.send_btn)

                layout.addWidget(self.output_display)
                layout.addLayout(input_layout)

            elif index == 6:  # Interface page
                layout.addWidget(QLabel(f"This is the {name} page"))
                self.interface_table = QTableWidget()
                layout.addWidget(self.interface_table)
            else:
                layout.addWidget(QLabel(f"This is the {name} page"))
                layout.addStretch()


            page.setLayout(layout)
            self.page_layout.addWidget(page)

        # === SERIAL DISPLAY ADDED TO "Setup" PAGE ===
        self.setup_serial_ui(self.page_layout.widget(0))

        # === LAYOUT COMPOSITION ===
        body_layout = QHBoxLayout()
        sidebar_frame = QFrame()
        sidebar_frame.setLayout(self.sidebar)
        sidebar_frame.setFixedWidth(160)

        right_frame = QFrame()
        right_frame.setLayout(self.page_layout)

        body_layout.addWidget(sidebar_frame)
        body_layout.addWidget(right_frame)

        # === MAIN LAYOUT ===
        main_layout = QVBoxLayout()
        main_layout.addLayout(header_layout)
        main_layout.addLayout(body_layout)
        self.setLayout(main_layout)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial_data)

    def setup_serial_ui(self, page):
        layout = page.layout()

        self.output_display = QTextEdit()
        self.output_display.setReadOnly(True)

        self.input_line = QLineEdit()
        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self.send_data)
    
        #Timestamp Button
        self.timestamp_enabled = True
        self.timestamp_btn = QPushButton("⏱ Timestamp: ON")
        self.timestamp_btn.setCheckable(True)
        self.timestamp_btn.setChecked(True)
        self.timestamp_btn.clicked.connect(self.toggle_timestamp)
        
        #Parse Button
        self.parse_btn = QPushButton("Parse Data")
        self.parse_btn.clicked.connect(self.open_parse_window)
        layout.addWidget(self.parse_btn)


        input_layout = QHBoxLayout()
        input_layout.addWidget(self.input_line)
        input_layout.addWidget(self.send_btn)

        layout.addWidget(self.timestamp_btn)
        layout.addWidget(self.output_display)
        layout.addLayout(input_layout)

    def refresh_ports(self):
        self.port_box.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_box.addItem(port.device)
        if not ports:
            self.show_error("No serial devices found. Please check connections.")

    def toggle_timestamp(self):
        self.timestamp_enabled = self.timestamp_btn.isChecked()
        if self.timestamp_enabled:
            self.timestamp_btn.setText("⏱ Timestamp: ON")
        else:
            self.timestamp_btn.setText("⏱ Timestamp: OFF")


    def toggle_connection(self):
        if self.serial_port.is_open:
            self.serial_port.close()
            self.connect_btn.setText("Connect")
            self.timer.stop()
        else:
            try:
                self.serial_port.port = self.port_box.currentText()
                self.serial_port.baudrate = int(self.baudrate_box.currentText())
                self.serial_port.open()
                self.connect_btn.setText("Disconnect")
                self.timer.start(100)
            except Exception as e:
                self.show_error(f"Failed to connect:\n{str(e)}")
                self.connect_btn.setText("Connect")

    def read_serial_data(self):
        if self.serial_port.is_open and self.serial_port.in_waiting:
            try:
                data = self.serial_port.readline().decode().strip()
                if self.timestamp_enabled:
                    now = datetime.now().strftime("[%H:%M:%S] ")
                    if self.timestamp_enabled:
                        now = datetime.now().strftime("[%H:%M:%S]")
                        colored = f'<span style="color:#00AAFF;">{now}</span> {data}'
                        self.output_display.append(colored)
                    else:
                        self.output_display.append(data)
                else:
                    self.output_display.append(data)

            except Exception as e:
                self.output_display.append(f"Read Error: {str(e)}")

    def send_data(self):
        if self.serial_port.is_open:
            text = self.input_line.text()
            self.serial_port.write((text + "\n").encode())
            self.output_display.append(f"Sent: {text}")
            self.input_line.clear()
            
    def open_parse_window(self):
        self.parse_window = ParseWindow(self)
        self.parse_window.show()

    def show_error(self, message):
        error_box = QMessageBox()
        error_box.setIcon(QMessageBox.Critical)
        error_box.setWindowTitle("Error")
        error_box.setText(message)
        error_box.exec_()

    def update_interface_table(self, fields):
        table = self.interface_table
        row_count = table.rowCount()
        table.insertRow(row_count)
        table.setColumnCount(len(fields))
        if hasattr(self.parent, 'update_interface_table'):
            self.parent.update_interface_table(fields)


        if table.rowCount() > 100:
            table.removeRow(0)


class ParseWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Parse Serial Data")
        self.setMinimumSize(600, 400)
        self.parent = parent
        self.latest_data = ""
        
        self.init_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_table)
        self.timer.start(500)

    def init_ui(self):
        layout = QVBoxLayout()

        # Radio buttons
        self.delim_radio = QRadioButton("Delimited")
        self.fixed_radio = QRadioButton("Fixed Width")
        self.delim_radio.setChecked(True)

        radio_group = QHBoxLayout()
        radio_group.addWidget(self.delim_radio)
        radio_group.addWidget(self.fixed_radio)
        layout.addLayout(radio_group)

        # Delimiter input
        delim_layout = QHBoxLayout()
        self.delim_input = QLineEdit(",")
        delim_layout.addWidget(QLabel("Delimiter:"))
        delim_layout.addWidget(self.delim_input)
        layout.addLayout(delim_layout)

        # Fixed width input
        self.width_input = QLineEdit("5,5,5")  # default widths
        width_layout = QHBoxLayout()
        width_layout.addWidget(QLabel("Fixed Widths (comma-separated):"))
        width_layout.addWidget(self.width_input)
        layout.addLayout(width_layout)

        # Sample preview
        self.sample_display = QTextEdit()
        self.sample_display.setReadOnly(True)
        layout.addWidget(QLabel("Sample Stream:"))
        layout.addWidget(self.sample_display)

        # Parsed table
        self.table = QTableWidget()
        layout.addWidget(QLabel("Parsed Output:"))
        layout.addWidget(self.table)

        self.setLayout(layout)

    def update_table(self):
        if self.parent.serial_port.is_open and self.parent.serial_port.in_waiting:
            try:
                line = self.parent.serial_port.readline().decode().strip()
                self.latest_data = line
                self.sample_display.append(line)

                if self.delim_radio.isChecked():
                    delimiter = self.delim_input.text()
                    fields = line.split(delimiter)
                else:
                    widths = [int(w) for w in self.width_input.text().split(",")]
                    fields = []
                    i = 0
                    for w in widths:
                        fields.append(line[i:i+w])
                        i += w

                self.table.setRowCount(1)
                self.table.setColumnCount(len(fields))
                for i, field in enumerate(fields):
                    self.table.setItem(0, i, QTableWidgetItem(field))

            except Exception as e:
                self.sample_display.append(f"Parse Error: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SerialApp()
    window.resize(1000, 600)
    window.show()
    sys.exit(app.exec_())
