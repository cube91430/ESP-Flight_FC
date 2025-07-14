import sys
import serial
import serial.tools.list_ports
import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QTextEdit, QFrame,
    QCheckBox, QStackedWidget, QWidget, QVBoxLayout, QLabel, QPushButton, QTextEdit,
    QRadioButton, QHBoxLayout, QComboBox, QLineEdit, QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import Qt, QTimer


class ParseWindow(QWidget):
    def __init__(self, sample_data):
        super().__init__()
        
        self.setWindowTitle("Parse Serial Data")
        self.setGeometry(300, 300, 600, 500)

        self.sample_data = sample_data.splitlines()
        self.parsed_rows = []

        # Layouts
        self.layout = QVBoxLayout()
        self.option_layout = QHBoxLayout()
        self.delim_layout = QHBoxLayout()

        # Radio buttons for parsing mode
        self.delim_radio = QRadioButton("Delimited")
        self.fixed_radio = QRadioButton("Fixed Width")
        self.delim_radio.setChecked(True)
        self.delim_radio.toggled.connect(self.toggle_delimiter_options)

        self.option_layout.addWidget(QLabel("Parsing Mode:"))
        self.option_layout.addWidget(self.delim_radio)
        self.option_layout.addWidget(self.fixed_radio)

        self.layout.addLayout(self.option_layout)

        # Delimiter options
        self.delim_combo = QComboBox()
        self.delim_combo.addItems(["Comma (,)", "Semicolon (;)", "Tab (\\t)", "Space", "Custom"])
        self.custom_delim_input = QLineEdit()
        self.custom_delim_input.setPlaceholderText("Enter custom delimiter")
        self.custom_delim_input.setEnabled(False)
        self.delim_combo.currentIndexChanged.connect(self.delimiter_changed)

        self.parse_button = QPushButton("Parse")
        self.parse_button.clicked.connect(self.do_parse)

        self.delim_layout.addWidget(QLabel("Delimiter:"))
        self.delim_layout.addWidget(self.delim_combo)
        self.delim_layout.addWidget(self.custom_delim_input)
        self.delim_layout.addWidget(self.parse_button)

        self.layout.addLayout(self.delim_layout)

        # Sample text preview
        self.layout.addWidget(QLabel("Sample Data:"))
        self.sample_text = QTextEdit()
        self.sample_text.setReadOnly(True)
        self.sample_text.setText(sample_data)
        self.layout.addWidget(self.sample_text)

        # Parsed table output
        self.table = QTableWidget()
        self.layout.addWidget(QLabel("Parsed Data:"))
        self.layout.addWidget(self.table)

        self.setLayout(self.layout)
        layout = QVBoxLayout()

        self.delimited_btn = QPushButton("Delimited")
        self.fixed_width_btn = QPushButton("Fixed Width")

        layout.addWidget(QLabel("Choose Parsing Method:"))
        layout.addWidget(self.delimited_btn)
        layout.addWidget(self.fixed_width_btn)

        layout.addSpacing(10)
        layout.addWidget(QLabel("Sample Data:"))

        self.sample_text = QTextEdit()
        self.sample_text.setReadOnly(True)
        self.sample_text.setText(sample_data)
        layout.addWidget(self.sample_text)

        self.setLayout(layout)

    def toggle_delimiter_options(self):
        enabled = self.delim_radio.isChecked()
        self.delim_combo.setEnabled(enabled)
        self.custom_delim_input.setEnabled(enabled and self.delim_combo.currentText() == "Custom")

    def delimiter_changed(self, index):
        self.custom_delim_input.setEnabled(self.delim_combo.currentText() == "Custom")

    def get_selected_delimiter(self):
        text = self.delim_combo.currentText()
        if text.startswith("Comma"):
            return ","
        elif text.startswith("Semicolon"):
            return ";"
        elif text.startswith("Tab"):
            return "\t"
        elif text.startswith("Space"):
            return " "
        elif text.startswith("Custom"):
            return self.custom_delim_input.text()
        return None

    def do_parse(self):
        self.parsed_rows.clear()
        self.table.clear()
        if self.delim_radio.isChecked():
            delimiter = self.get_selected_delimiter()
            for line in self.sample_data:
                self.parsed_rows.append(line.split(delimiter))
        else:
            # Fixed width split (example: 10-character columns)
            width = 10  # You can make this dynamic later
            for line in self.sample_data:
                chunks = [line[i:i+width].strip() for i in range(0, len(line), width)]
                self.parsed_rows.append(chunks)

        self.show_parsed_data()

    def show_parsed_data(self):
        max_cols = max(len(row) for row in self.parsed_rows)
        self.table.setRowCount(len(self.parsed_rows))
        self.table.setColumnCount(max_cols)

        for i, row in enumerate(self.parsed_rows):
            for j, item in enumerate(row):
                self.table.setItem(i, j, QTableWidgetItem(item))

class FlightDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Flight Controller Dashboard")
        self.setGeometry(200, 200, 1000, 600)

        self.serial_port = None
        self.is_connected = False

        self.init_ui()

    def init_ui(self):
        main_widget = QWidget()
        main_layout = QVBoxLayout()

        # --- Top Navbar ---
        navbar = QHBoxLayout()
        self.port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.refresh_ports()

        self.baud_label = QLabel("Baudrate:")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "57600", "115200", "250000"])

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)

        navbar.addWidget(self.port_label)
        navbar.addWidget(self.port_combo)
        navbar.addWidget(self.baud_label)
        navbar.addWidget(self.baud_combo)
        navbar.addWidget(self.connect_btn)

        # --- Main content layout ---
        content_layout = QHBoxLayout()

        # --- Sidebar Navigation (2D Buttons) ---
        sidebar = QVBoxLayout()
        sidebar.setAlignment(Qt.AlignTop)

        self.setup_btn = QPushButton("Setup")
        self.visual_btn = QPushButton("Visualization")
        self.data_btn = QPushButton("Data")

        for btn in [self.setup_btn, self.visual_btn, self.data_btn]:
            btn.setFlat(True)
            btn.setStyleSheet("QPushButton { border: none; padding: 10px; text-align: left; }"
                              "QPushButton:hover { background-color: #ddd; }")

        self.setup_btn.clicked.connect(lambda: self.page_stack.setCurrentIndex(0))
        self.visual_btn.clicked.connect(lambda: self.page_stack.setCurrentIndex(1))
        self.data_btn.clicked.connect(lambda: self.page_stack.setCurrentIndex(2))

        sidebar.addWidget(self.setup_btn)
        sidebar.addWidget(self.visual_btn)
        sidebar.addWidget(self.data_btn)

        sidebar_frame = QFrame()
        sidebar_frame.setFrameShape(QFrame.StyledPanel)
        sidebar_frame.setMinimumWidth(150)
        sidebar_frame.setLayout(sidebar)
        sidebar_frame.setStyleSheet("background-color: #f4f4f4;")

        # --- Pages Stack ---
        self.page_stack = QStackedWidget()

        self.setup_page = self.create_setup_page()
        self.visual_page = self.create_visual_page()
        self.data_page = self.create_data_page()

        self.page_stack.addWidget(self.setup_page)
        self.page_stack.addWidget(self.visual_page)
        self.page_stack.addWidget(self.data_page)

        content_layout.addWidget(sidebar_frame)
        content_layout.addWidget(self.page_stack)

        main_layout.addLayout(navbar)
        main_layout.addLayout(content_layout)

        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Timer for reading serial
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
    
    def show_parse_window(self):
        full_text = self.serial_display.toPlainText().strip()
        lines = full_text.splitlines()[-10:]  # Last 10 lines
        sample_data = "\n".join(lines)
        self.parse_window = ParseWindow(sample_data)
        self.parse_window.show()


        
    def create_data_page(self):
        page = QWidget()
        layout = QVBoxLayout()

        # --- Options row ---
        options_layout = QHBoxLayout()
        self.autoscroll_checkbox = QCheckBox("Autoscroll")
        self.timestamp_checkbox = QCheckBox("Add Timestamp")
        self.autoscroll_checkbox.setChecked(True)

        self.parse_button = QPushButton("Parse")
        self.parse_button.clicked.connect(self.show_parse_window)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(lambda: self.serial_display.clear())

        options_layout.addWidget(self.autoscroll_checkbox)
        options_layout.addWidget(self.timestamp_checkbox)
        options_layout.addStretch()
        options_layout.addWidget(self.parse_button)
        options_layout.addWidget(self.clear_button)

        # --- Serial data display ---
        self.serial_display = QTextEdit()
        self.serial_display.setReadOnly(True)

        layout.addLayout(options_layout)
        layout.addWidget(self.serial_display)

        page.setLayout(layout)
        return page


    def create_setup_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("This is the Setup Page. Add setup widgets here."))
        page.setLayout(layout)
        return page

    def create_visual_page(self):
        page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("This is the Visualization Page. Add plots or 3D visual here."))
        page.setLayout(layout)
        return page

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def toggle_connection(self):
        if not self.is_connected:
            self.connect_device()
        else:
            self.disconnect_device()

    def connect_device(self):
        port = self.port_combo.currentText()
        baudrate = int(self.baud_combo.currentText())
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.connect_btn.setText("Disconnect")
            self.is_connected = True
            self.timer.start(100)
        except serial.SerialException as e:
            self.serial_display.append(f"[ERROR] Could not open port: {e}")

    def disconnect_device(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connect_btn.setText("Connect")
        self.is_connected = False
        self.timer.stop()

    def read_serial(self):
        if self.serial_port and self.serial_port.in_waiting:
            try:
                raw_data = self.serial_port.readline().decode("utf-8").strip()
                if self.timestamp_checkbox.isChecked():
                    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
                    data = f"[{timestamp}] {raw_data}"
                else:
                    data = raw_data

                self.serial_display.append(data)

                if self.autoscroll_checkbox.isChecked():
                    self.serial_display.moveCursor(self.serial_display.textCursor().End)
            except Exception as e:
                self.serial_display.append(f"[READ ERROR] {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = FlightDashboard()
    window.show()
    sys.exit(app.exec_())
