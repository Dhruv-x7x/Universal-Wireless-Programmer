import tkinter as tk
from tkinter import messagebox, ttk, filedialog, scrolledtext
import serial
import os
import time
import serial.tools.list_ports
import threading
import re
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from collections import deque
import subprocess
import tempfile
import os.path

# Initialize variables
ser = None  # Single port for both transmission and receiving
parsed_lines = []  # Store parsed HEX lines
loaded_file_name = ""  # Track loaded HEX file name
serial_monitor_active = False  # Track if serial monitor is active
plotting_active = False  # Track if plotting is active
passthrough_active = False  # Track if passthrough mode is active
signature_verified = False  # Track if signature has been verified
total_size = 0
# Serial data buffers and plot data
serial_data = ""
plot_data = {
    "time": deque(maxlen=100),
    "series": [],      # Will hold a list of deques for each series
    "colors": ['b-', 'r-', 'g-', 'y-', 'm-', 'c-', 'k-', 'orange', 'purple', 'brown']  # Colors for different series
}
# Dictionary of board signatures
BOARD_SIGNATURES = {
    "Arduino UNO": {"name": "Arduino UNO R3", "sig": "1E,95,0F"},
    "Arduino Nano": {"name": "ARDUINO NANO", "sig": "1E,95,0F"},
    "Arduino Mega": {"name": "ARDUINO MEGA", "sig": "1E,98,01"},
    "STM32F103": {"name": "STM32F103", "sig": "1E,90,20"}
}

# Updated colors for dark theme
COLORS = {
    'background': '#1e1e1e',      # Dark background
    'secondary_bg': '#2d2d2d',    # Slightly lighter background for contrast
    'button_bg': '#424242',       # Dark grey for buttons
    'button_fg': '#E0E0E0',       # Light grey for button text
    'button_active_bg': '#616161', # Lighter grey for button hover
    'text': '#E0E0E0',           # Light grey for general text
    'accent': '#BB86FC',         # Light purple for accents
    'treeview_bg': '#2d2d2d',    # Dark background for treeview (initial dark)
    'treeview_selected': '#424242', # Selected row color
    'header_bg': '#2d2d2d',      # Header background
    'danger': '#CF6679',         # Red for danger/error
    'success': '#03DAC6',        # Teal for success
    'dropdown_bg': '#424242',    # Same as button background
    'dropdown_fg': '#E0E0E0',    # Same as button text
    'label_fg': '#BB86FC',       # Light purple for label text
    'combobox_fg': '#000000',    # Set combobox text to black
    'treeview_fg': '#E0E0E0',    # Light grey text for treeview
    'tab_selected': '#BB86FC',   # Selected tab color
    'tab_bg': '#333333',         # Tab background
}

selected_board = None  # Track selected board
current_view = "logs"  # Track current view: "logs", "monitor", or "plot"

# Function to create a new top-level window with maximize/minimize
def create_detachable_window(title, parent_frame):
    window = tk.Toplevel(root)
    window.title(title)
    window.geometry("800x600")
    window.configure(bg=COLORS['background'])
    
    # Create a frame to hold the content
    content_frame = ttk.Frame(window, style='Custom.TFrame')
    content_frame.pack(fill='both', expand=True, padx=10, pady=10)
    
    # Move the parent_frame to the new window
    parent_frame.pack_forget()
    parent_frame.pack(in_=content_frame, fill='both', expand=True)
    
    # Function to restore the frame back to main window
    def on_close():
        parent_frame.pack_forget()
        if current_view == "logs":
            parent_frame.pack(in_=log_container, fill='both', expand=True)
            show_logs()
        elif current_view == "monitor":
            parent_frame.pack(in_=monitor_container, fill='both', expand=True)
            show_monitor()
        elif current_view == "plot":
            parent_frame.pack(in_=plot_container, fill='both', expand=True)
            show_plot()
        window.destroy()
    
    window.protocol("WM_DELETE_WINDOW", on_close)
    return window

def create_button(parent, text, command, state='normal', width=None):
    """Create a custom tk button with consistent styling"""
    button = tk.Button(parent,
                      text=text,
                      command=command,
                      bg=COLORS['button_bg'],
                      fg=COLORS['button_fg'],
                      activebackground=COLORS['button_active_bg'],
                      activeforeground=COLORS['button_fg'],
                      relief='flat',
                      state=state,
                      font=('Helvetica', 9),
                      padx=10,
                      pady=5,
                      width=width)
    return button

def create_tab_button(parent, text, command, is_active=False):
    """Create a tab-like button for switching views"""
    bg_color = COLORS['tab_selected'] if is_active else COLORS['tab_bg']
    fg_color = COLORS['combobox_fg'] if is_active else COLORS['button_fg']
    
    button = tk.Button(parent,
                      text=text,
                      command=command,
                      bg=bg_color,
                      fg=fg_color,
                      activebackground=COLORS['tab_selected'],
                      activeforeground=COLORS['combobox_fg'],
                      relief='flat',
                      font=('Helvetica', 10, 'bold'),
                      padx=15,
                      pady=8,
                      width=15)
    return button

def create_custom_style():
    """Create custom styling for widgets"""
    style = ttk.Style()
    
    # Configure main theme
    style.configure('Custom.TFrame', background=COLORS['background'])
    
    # Custom label frame style
    style.configure('Custom.TLabelframe', background=COLORS['background'])
    style.configure('Custom.TLabelframe.Label',
                   foreground=COLORS['accent'],
                   font=('Helvetica', 10, 'bold'),
                   background=COLORS['background'])
    
    # Custom Treeview Style (Dark Background)
    style.configure('Custom.Treeview',
               background=COLORS['treeview_bg'],  # Dark background
               fieldbackground=COLORS['treeview_bg'],  # Fix empty space color
               foreground=COLORS['treeview_fg'],  # Light text
               borderwidth=0)

    # Fix white space inside the Treeview
    style.map('Treeview',
          background=[('disabled', COLORS['button_fg'])])  # Force dark background

    # Custom Treeview Heading Style (Dark)
    style.configure('Custom.Treeview.Heading',
               background=COLORS['button_fg'],  # Dark background for heading
               foreground=COLORS['combobox_fg'],  # Light text color
               font=('Helvetica', 10, 'bold'),
               relief='flat')
    
    # Custom combobox style (gray background)
    style.configure('Custom.TCombobox',
                   background=COLORS['dropdown_bg'],
                   fieldbackground=COLORS['dropdown_bg'],
                   foreground=COLORS['combobox_fg'],  # Normal black text in combobox
                   selectbackground=COLORS['treeview_selected'],
                   selectforeground=COLORS['text'])
    
    # Configure label style
    style.configure('Custom.TLabel',
                   background=COLORS['background'],
                   foreground=COLORS['text'])
    
    # Configure status label styles
    style.configure('Connected.TLabel',
                   background=COLORS['background'],
                   foreground=COLORS['success'],
                   font=('Helvetica', 9, 'bold'))
    style.configure('Disconnected.TLabel',
                   background=COLORS['background'],
                   foreground=COLORS['danger'],
                   font=('Helvetica', 9, 'bold'))
    
    # Configure the Text widget style
    style.configure('Serial.TFrame',
                   background=COLORS['secondary_bg'])

def compile_arduino_code(code, board_type="arduino:avr:uno", output_dir=None):
    """
    Compile Arduino code to a HEX file using Arduino CLI
    
    Parameters:
    - code: The Arduino C/C++ code to compile
    - board_type: Arduino board type (default is Arduino UNO)
    - output_dir: Directory to save the compiled files (temporary if None)
    
    Returns:
    - Tuple (success, hex_file_path or error_message)
    """
    default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "compiled_hex")
    if not os.path.exists(default_dir):
        os.makedirs(default_dir)
    
    # Create a timestamp-based folder to avoid overwriting
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(default_dir, f"sketch_{timestamp}")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Create a temporary sketch file
    sketch_dir = os.path.join(output_dir, "sketch")
    os.makedirs(sketch_dir, exist_ok=True)
    sketch_path = os.path.join(sketch_dir, "sketch.ino")
    
    with open(sketch_path, "w") as f:
        f.write(code)
    
    # Get the board type in the right format for Arduino CLI
    board_mapping = {
        "Arduino UNO": "arduino:avr:uno",
        "Arduino Nano": "arduino:avr:nano",
        "Arduino Mega": "arduino:avr:mega",
        "Arduino Leonardo": "arduino:avr:leonardo"
    }
    
    cli_board_type = board_mapping.get(board_type, board_type)
    
    try:
        # Compile the sketch using Arduino CLI
        arduino_cli_path = r"C:\Users\Admin\Downloads\arduino-cli_1.2.0_Windows_64bit\arduino-cli.exe"
        
        # Compile the sketch using Arduino CLI with the full path
        result = subprocess.run(
            [arduino_cli_path, "compile", "--fqbn", cli_board_type, sketch_path, "--output-dir", output_dir],
            capture_output=True,
            text=True,
            check=False
        )
        
        if result.returncode != 0:
            return False, f"Compilation failed:\n{result.stderr}"
        
        # Find the hex file
        hex_file = os.path.join(output_dir, "sketch.ino.hex")
        if not os.path.exists(hex_file):
            # For some boards, the filename might be different
            hex_file = os.path.join(output_dir, "sketch.ino.with_bootloader.hex")
            if not os.path.exists(hex_file):
                return False, "Compilation succeeded but HEX file not found"
        
        return True, hex_file
        
    except Exception as e:
        return False, f"Error during compilation: {str(e)}"

def check_arduino_cli():
    """Check if Arduino CLI is installed and accessible"""
    try:
        # Try with full path
        cli_path = r"C:\Users\Admin\Downloads\arduino-cli_1.2.0_Windows_64bit\arduino-cli.exe"
        
        if os.path.exists(cli_path):
            result = subprocess.run(
                [cli_path, "version"],
                capture_output=True,
                text=True,
                check=False
            )
            if result.returncode == 0:
                # Store the path for later use
                global arduino_cli_path
                arduino_cli_path = cli_path
                return True
            else:
                return False
        else:
            return False
            
    except Exception as e:
        add_log_message(f"Error checking Arduino CLI: {str(e)}")
        return False

def save_code(code):
    """Save the Arduino code to a file"""
    file_path = filedialog.asksaveasfilename(
        defaultextension=".ino",
        filetypes=[("Arduino Sketch", ".ino"), ("All Files", ".*")]
    )
    if not file_path:
        return
    
    try:
        with open(file_path, "w") as f:
            f.write(code)
        add_log_message(f"Code saved to: {file_path}")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to save code:\n{e}")
        add_log_message(f"Error saving code: {str(e)}")

def load_code(editor):
    """Load Arduino code from a file into the editor"""
    file_path = filedialog.askopenfilename(
        filetypes=[("Arduino Sketch", ".ino"), ("C/C++ Files", ".cpp .c *.h"), ("All Files", ".*")]
    )
    if not file_path:
        return
    
    try:
        with open(file_path, "r") as f:
            code = f.read()
        
        editor.delete(1.0, tk.END)
        editor.insert(tk.END, code)
        add_log_message(f"Code loaded from: {file_path}")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to read file:\n{e}")
        add_log_message(f"Error loading code: {str(e)}")

def pop_out_editor(editor_frame):
    """Open code editor in a new window"""
    create_detachable_window("Arduino Code Editor", editor_frame)

def compile_code(code):
    """Compile the Arduino code to a hex file"""
    global selected_board, loaded_file_name
    
    # First, check if Arduino CLI is installed
    if not check_arduino_cli():
        messagebox.showerror(
            "Arduino CLI Not Found",
            "Arduino CLI is not installed or not in the PATH.\n"
            "Please install Arduino CLI from https://arduino.github.io/arduino-cli/."
        )
        add_log_message("Error: Arduino CLI not found")
        return
    
    # Get the board type
    if selected_board:
        board_type = selected_board
    else:
        board_type = "Arduino UNO"  # Default to UNO if none selected
    
    add_log_message(f"Compiling code for {board_type}...")
    
    # Show a progress indicator
    progress_var.set(10)
    root.update_idletasks()
    
    # Compile the code
    success, result = compile_arduino_code(code, board_type)
    
    if success:
        # Loading the compiled hex file
        hex_file_path = result
        add_log_message(f"Compilation successful: {hex_file_path}")
        
        # Update progress
        progress_var.set(75)
        root.update_idletasks()
        
        try:
            # Load the hex file into the GUI
            with open(hex_file_path, "r") as file:
                lines = file.readlines()

            # Clear existing data
            for row in tree.get_children():
                tree.delete(row)

            global parsed_lines
            parsed_lines = []  # Reset HEX data list

            for line in lines:
                parsed_data = parse_hex_line(line)
                if parsed_data:
                    parsed_lines.append(line.strip())
                    tree.insert("", "end", values=(
                        parsed_data["Byte Count"],
                        parsed_data["Address"],
                        parsed_data["Record Type"],
                        parsed_data["Data"],
                        parsed_data["Checksum"]
                    ))

            loaded_file_name = os.path.basename(hex_file_path)
            label_loaded_file.config(text=f"Loaded File: {loaded_file_name}")
            
            # Complete progress
            progress_var.set(100)
            root.update_idletasks()
            
            # Success message
            messagebox.showinfo("Success", f"Compilation successful. HEX file loaded and ready for programming.")
            add_log_message("HEX file loaded and ready for programming")
            
            # Switch to the log view
            show_logs()
            
        except Exception as e:
            progress_var.set(0)
            messagebox.showerror("Error", f"Failed to load compiled HEX file:\n{e}")
            add_log_message(f"Error loading compiled HEX file: {str(e)}")
    else:
        # Compilation failed
        progress_var.set(0)
        messagebox.showerror("Compilation Failed", result)
        add_log_message(f"Compilation failed: {result}")

def update_tab_buttons():
    """Update the tab buttons based on the current view"""
    global current_view
    
    # Reset all buttons to inactive state
    btn_logs.config(bg=COLORS['tab_bg'], fg=COLORS['button_fg'])
    btn_monitor.config(bg=COLORS['tab_bg'], fg=COLORS['button_fg'])
    btn_plot.config(bg=COLORS['tab_bg'], fg=COLORS['button_fg'])
    btn_editor.config(bg=COLORS['tab_bg'], fg=COLORS['button_fg'])
    
    # Set the active button
    if current_view == "logs":
        btn_logs.config(bg=COLORS['tab_selected'], fg=COLORS['combobox_fg'])
    elif current_view == "monitor":
        btn_monitor.config(bg=COLORS['tab_selected'], fg=COLORS['combobox_fg'])
    elif current_view == "plot":
        btn_plot.config(bg=COLORS['tab_selected'], fg=COLORS['combobox_fg'])
    elif current_view == "editor":
        btn_editor.config(bg=COLORS['tab_selected'], fg=COLORS['combobox_fg'])

def parse_hex_line(line):
    """Parses a HEX file line into components."""
    if not line.startswith(":"):
        return None

    line = line.strip()
    byte_count = int(line[1:3], 16)
    address = line[3:7]
    record_type = line[7:9]
    data = line[9:-2]
    checksum = line[-2:]

    return {
        "Byte Count": byte_count,
        "Address": address,
        "Record Type": record_type,
        "Data": data,
        "Checksum": checksum
    }

def load_hex_file():
    """Loads HEX file and parses it."""
    global loaded_file_name
    file_path = filedialog.askopenfilename(filetypes=[("HEX Files", ".hex"), ("All Files", ".*")])

    if not file_path:
        return  

    try:
        with open(file_path, "r") as file:
            lines = file.readlines()

        # Clear existing data
        for row in tree.get_children():
            tree.delete(row)

        global parsed_lines
        parsed_lines = []  # Reset HEX data list

        for line in lines:
            parsed_data = parse_hex_line(line)
            if parsed_data:
                parsed_lines.append(line.strip())
                tree.insert("", "end", values=(
                    parsed_data["Byte Count"],
                    parsed_data["Address"],
                    parsed_data["Record Type"],
                    parsed_data["Data"],
                    parsed_data["Checksum"]
                ))

        loaded_file_name = os.path.basename(file_path)
        label_loaded_file.config(text=f"Loaded File: {loaded_file_name}")
        
        # Add log entry
        add_log_message(f"Loaded HEX file: {loaded_file_name}")

    except Exception as e:
        messagebox.showerror("Error", f"Failed to read file:\n{e}")
        add_log_message(f"Error loading file: {str(e)}")

def select_board():
    """Handle board selection"""
    global selected_board
    selected = board_var.get()
    if selected in BOARD_SIGNATURES:
        selected_board = selected
        messagebox.showinfo("Success", f"Selected board: {selected}")
        add_log_message(f"Selected board: {selected}")
    else:
        messagebox.showerror("Error", "Please select a valid board!")
        add_log_message("Error: No valid board selected")

def detect_ports():
    """Detects USB-to-UART modules and populates dropdown."""
    ports = list(serial.tools.list_ports.comports())
    usb_ports = []

    for port in ports:
        port_name = port.device
        description = port.description

        if "USB" in description or "UART" in description or "CP210" in description or "CH340" in description:
            usb_ports.append(f"{port_name} - {description}")

    com_dropdown['values'] = usb_ports

    add_log_message(f"Detected {len(usb_ports)} USB ports")

def update_status_label():
    """Update the UART connection status label"""
    global ser
    
    # Update status
    if ser and ser.is_open:
        status_label.configure(style='Connected.TLabel')
        status_label.config(text="Connected")
    else:
        status_label.configure(style='Disconnected.TLabel')
        status_label.config(text="Disconnected")

def connect_serial():
    """Establish UART connection for both TX and RX."""
    global ser, serial_monitor_active, signature_verified
    
    # Get port info
    port_info = com_var.get()
    if not port_info:
        messagebox.showerror("Error", "No COM port selected!")
        add_log_message("Error: No COM port selected")
        return
    port = port_info.split(" - ")[0]
    
    # Get baud rate
    baud = int(baud_var.get())

    # Close existing connection
    disconnect_serial()

    try:
        # Open connection
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        # Reset signature verification state
        signature_verified = False
        verify_indicator.configure(style='Disconnected.TLabel')
        verify_indicator.config(text="Signature: Not Verified")
        
        # Start monitoring thread
        serial_monitor_active = True
        rx_thread = threading.Thread(target=read_serial_data, daemon=True)
        rx_thread.start()
        
        messagebox.showinfo("Success", f"Connected to: {port} at {baud} baud.")
        add_log_message(f"Connected to: {port} at {baud} baud")
        update_status_label()
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to open port:\n{e}")
        add_log_message(f"Error connecting to port: {str(e)}")
        disconnect_serial()
        update_status_label()

def disconnect_serial():
    """Disconnect serial port."""
    global ser, serial_monitor_active, passthrough_active, signature_verified
    
    # Reset state flags
    passthrough_active = False
    serial_monitor_active = False
    signature_verified = False
    
    # Close port
    if ser and ser.is_open:
        ser.close()
    ser = None
    
    # Reset UI elements
    btn_passthrough.config(text="Serial Mode", bg=COLORS['button_bg'])
    btn_exit_passthrough.config(state='disabled')
    update_mode_indicator()
    
    # Reset verification status
    verify_indicator.configure(style='Disconnected.TLabel')
    verify_indicator.config(text="Signature: Not Verified")
    
    # Update status
    update_status_label()
    add_log_message("Disconnected serial port")

def clear_serial_monitor():
    """Clear the serial monitor text area"""
    serial_monitor.config(state=tk.NORMAL)
    serial_monitor.delete(1.0, tk.END)
    serial_monitor.config(state=tk.DISABLED)
    add_log_message("Serial monitor cleared")

def read_serial_data():
    """Background thread to continuously read data from serial port"""
    global ser, serial_monitor_active, passthrough_active
    
    while serial_monitor_active and ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    # Always process the data for commands/signatures regardless of mode
                    process_received_data(data)
                    
                    # Route the display based on passthrough mode flag
                    if passthrough_active:
                        # In passthrough mode, send to Serial Monitor
                        serial_monitor.config(state=tk.NORMAL)
                        serial_monitor.insert(tk.END, data + "\n")
                        serial_monitor.see(tk.END)
                        serial_monitor.config(state=tk.DISABLED)
                    else:
                        # In normal mode, send to Activity Log
                        add_log_message(f"STM: {data}")
                    
                    # Handle data for plotting if it matches a numerical pattern
                    process_for_plotting(data)
                
        except Exception as e:
            print(f"Serial read error: {str(e)}")
            add_log_message(f"Serial read error: {str(e)}")
            break
            
        time.sleep(0.01)  # Short delay to prevent CPU overuse

def process_received_data(data):
    """Process data received from the STM32"""
    global signature_verified
    
    # Check for signature response - look for "SIG:" pattern anywhere in the line
    # Only process signature if we haven't verified it yet
    if "SIG:" in data and not signature_verified:
        # Extract the signature using regex to get the exact format
        sig_match = re.search(r'SIG:([0-9A-F]{1,2},){2}[0-9A-F]{1,2}', data)
        if sig_match:
            # Get just the complete signature match
            sig_str = sig_match.group(0)
            # Strip the "SIG:" prefix
            received_sig = sig_str[4:]
            add_log_message(f"Extracted signature: {received_sig}")
            process_signature_response(received_sig)
        else:
            add_log_message(f"Could not extract valid signature format from: {data}")
    
    # Check for programming status messages
    elif "PROG:" in data:
        # Extract status part after PROG:
        status_match = re.search(r'PROG:(\w+)', data)
        if status_match:
            status = status_match.group(1)
            if status == "SUCCESS":
                add_log_message("Programming successful!")
                messagebox.showinfo("Success", "Programming completed successfully!")
            elif status == "FAIL":
                add_log_message("Programming failed")
                messagebox.showerror("Error", "Programming failed. Check connections and try again.")
            elif status.endswith("%"):
                # Update progress bar if it's a percentage
                try:
                    percent = int(status.strip("%"))
                    progress_var.set(percent)
                    root.update_idletasks()
                except ValueError:
                    pass
    
    # Handle hex data acknowledgments
    elif "HEX:" in data:
        status_match = re.search(r'HEX:(\w+)', data)
        if status_match:
            status = status_match.group(1)
            if status == "READY":
                # STM is ready to receive binary data - handled in send_hex_chunk
                add_log_message("STM ready to receive hex chunk")
            elif status == "ACK":
                add_log_message("Hex chunk acknowledged")
            elif status == "NAK":
                add_log_message("Error: Hex chunk not acknowledged")
            elif status == "ERR_SIZE" or status == "ERR_FORMAT":
                add_log_message(f"Error: {status}")
    
    if "Invalid file size" in data:
        add_log_message(f"Retrying with correct size: {total_size} bytes")
            
            # Retry sending the size information and binary data
        ser.write(size_bytes)
        add_log_message(f"Sent size information again: {total_size} bytes")
      
def process_signature_response(received_sig):
    """Process and verify the signature received from the STM32"""
    global selected_board, signature_verified
    
    add_log_message(f"Received signature: {received_sig}")
    
    # Clean the signature in case of any extraneous characters
    cleaned_sig = received_sig.strip()
    
    # Try to match with known board signatures
    matched_board = None
    for board, info in BOARD_SIGNATURES.items():
        if cleaned_sig.upper() == info["sig"].upper():  # Case-insensitive comparison
            matched_board = board
            break
    
    if matched_board:
        # We found a matching board
        if selected_board and selected_board == matched_board:
            # The selected board matches the signature
            add_log_message(f"Signature verified successfully! Confirmed {selected_board}")
            messagebox.showinfo("Success", f"Signature verification successful!\nBoard: {selected_board}")
        else:
            # We found a match but it's different from what user selected
            add_log_message(f"Signature matches {matched_board} instead of {selected_board or 'None'}")
            signature_verified = False
            # Ask user if they want to switch to the detected board
            if selected_board:
                response = messagebox.askyesno("Board Mismatch", 
                                f"Detected board: {matched_board}\n" + 
                                f"This is different from your selection: {selected_board}\n\n" +
                                f"Would you like to switch to the detected board?")
                if response:
                    # Update the selected board
                    selected_board = matched_board
                    board_var.set(matched_board)
                    add_log_message(f"Switched to detected board: {matched_board}")
            else:
                # No board was previously selected, just set it
                selected_board = matched_board
                board_var.set(matched_board)
                add_log_message(f"Set board to detected: {matched_board}")
                messagebox.showinfo("Board Detected", f"Detected board: {matched_board}")
            
        # Set the verification flag
        signature_verified = True
        
        # Update UI to show verification status
        verify_indicator.configure(style='Connected.TLabel')
        verify_indicator.config(text="Signature: Verified")
    else:
        # No matching signature found
        expected_sig = "unknown"
        if selected_board:
            expected_sig = BOARD_SIGNATURES[selected_board]["sig"]
        
        add_log_message(f"Signature verification failed! Expected: {expected_sig}, Received: {cleaned_sig}")
        
        # Ask if user wants to proceed anyway
        response = messagebox.askyesno("Signature Mismatch", 
                                     f"Signature verification failed!\nExpected: {expected_sig}\nReceived: {cleaned_sig}\n\n" +
                                     f"Would you like to proceed anyway? This might cause programming issues.")
        if response:
            # User wants to proceed despite signature mismatch
            signature_verified = True
            verify_indicator.configure(style='Connected.TLabel')
            verify_indicator.config(text="Signature: Override")
            add_log_message("User overrode signature verification")
        else:
            # User canceled, keep verification as failed
            signature_verified = False
            verify_indicator.configure(style='Disconnected.TLabel')
            verify_indicator.config(text="Signature: Not Verified")


def process_for_plotting(data):
    """Process data for real-time plotting with support for multiple values per line"""
    global plot_data, plotting_active
    
    if not plotting_active:
        return
        
    # Try to parse multiple comma/space/tab-separated values
    # First, look for typical formats: 
    # - Plain numbers: "100,200,300" or "100 200 300"
    # - Labeled values: "temp:100 humid:50" or "temp:100,humid:50"
    
    # Remove any labels for now (we'll just keep the values)
    cleaned_data = re.sub(r'[a-zA-Z_]+:', '', data)
    
    # Split by common separators (comma, space, tab)
    values = re.split(r'[,\s\t]+', cleaned_data)
    
    # Try to parse each value as a float
    numeric_values = []
    for val in values:
        try:
            num_val = float(val.strip())
            numeric_values.append(num_val)
        except ValueError:
            # Skip values that can't be parsed as floats
            pass
    
    # If we found numeric values
    if numeric_values and len(numeric_values) > 0:
        current_time = time.time()
        
        # Initialize the time if this is the first data point
        if len(plot_data["time"]) == 0:
            relative_time = 0
        else:
            relative_time = plot_data["time"][-1] + 0.1  # Approximate 100ms between points
        
        # Add the time point
        plot_data["time"].append(relative_time)
        
        # Make sure we have enough data series
        while len(plot_data["series"]) < len(numeric_values):
            plot_data["series"].append(deque(maxlen=100))
        
        # Add each value to its respective series
        for i, value in enumerate(numeric_values):
            plot_data["series"][i].append(value)
        
        # Update the plot
        update_plot()

def update_plot():
    """Update the real-time plot with multiple data series"""
    global plot_data
    
    # Clear the current plot
    ax.clear()
    
    # Plot each data series with its own color
    for i, series in enumerate(plot_data["series"]):
        if len(series) > 0:
            # Use modulo to cycle through colors if we have more series than colors
            color_index = i % len(plot_data["colors"])
            color = plot_data["colors"][color_index]
            
            # Plot this data series
            ax.plot(list(plot_data["time"])[-len(series):], list(series), color, label=f"Series {i+1}")
    
    # Add a legend if we have multiple series
    if len(plot_data["series"]) > 1:
        ax.legend(loc='upper left')
    
    # Set labels and title
    ax.set_title('Serial Data Plot')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Value')
    
    # Set grid
    ax.grid(True)
    
    # Auto-scale the y-axis to fit the data
    if any(len(series) > 0 for series in plot_data["series"]):
        all_values = [val for series in plot_data["series"] for val in series if len(series) > 0]
        if all_values:
            min_val = min(all_values)
            max_val = max(all_values)
            range_val = max_val - min_val
            
            # Add 10% padding to the top and bottom
            if range_val > 0:
                ax.set_ylim([min_val - range_val*0.1, max_val + range_val*0.1])
    
    # Redraw the canvas
    canvas.draw()

def toggle_plotting():
    """Toggle real-time plotting on/off"""
    global plotting_active, plot_data
    
    plotting_active = not plotting_active
    
    if plotting_active:
        # Clear plot data when starting
        plot_data = {
            "time": deque(maxlen=100),
            "series": [],  # Empty list of data series
            "colors": ['b-', 'r-', 'g-', 'y-', 'm-', 'c-', 'k-', 'orange', 'purple', 'brown']  # Keep the colors
        }
        btn_toggle_plot.config(text="Stop Plotting")
        add_log_message("Real-time plotting started")
    else:
        btn_toggle_plot.config(text="Start Plotting")
        add_log_message("Real-time plotting stopped")

def check_signature():
    """Send signature verification command to the STM32"""
    global ser, signature_verified
    
    if ser is None or not ser.is_open:
        messagebox.showerror("Error", "UART connection not established!")
        add_log_message("Error: No connection for signature check")
        return

    try:
        # Reset the signature verification state
        signature_verified = False
        
        # Update verification indicator
        verify_indicator.configure(style='Disconnected.TLabel')
        verify_indicator.config(text="Signature: Checking...")
        
        # Update UI to show we're checking
        add_log_message("Requesting signature verification...")
        
        # Send the signature verification command
        ser.write(b"s\r\n")
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to send signature request:\n{e}")
        add_log_message(f"Error sending signature request: {str(e)}")
        
        # Reset verification indicator on error
        verify_indicator.configure(style='Disconnected.TLabel')
        verify_indicator.config(text="Signature: Error")

def send_hex_data():
    """Send the entire loaded hex file to the STM32 in one continuous transmission"""
    global ser, parsed_lines, signature_verified
    
    if ser is None or not ser.is_open:
        messagebox.showerror("Error", "UART connection not established!")
        add_log_message("Error: No connection for programming")
        return

    if not parsed_lines:
        messagebox.showerror("Error", "No HEX file loaded!")
        add_log_message("Error: No HEX file loaded for programming")
        return
    
    # Check if signature has been verified
    if not signature_verified:
        response = messagebox.askyesno("Warning", 
                                      "Board signature has not been verified. "
                                      "Do you want to continue with programming anyway?")
        if not response:
            add_log_message("Programming canceled - signature not verified")
            return
        add_log_message("User chose to continue without signature verification")

    try:
        # Reset progress bar
        progress_var.set(0)
        
        # Send programming start command (simple 'p' command)
        add_log_message("Sending programming start command 'p'")
        ser.write(b"p\r\n")  
        
        # Use a fixed delay to allow STM32 to enter programming mode
        time.sleep(2.0)
        add_log_message("Assuming STM32 is ready for programming after delay")
        
        # Process the hex file into binary format
        binary_data = process_hex_to_binary()
        
        if not binary_data or len(binary_data) == 0:
            messagebox.showerror("Error", "Failed to process hex file into binary data")
            add_log_message("Error: Failed to process hex file into binary data")
            return
            
        # Log the total size
        total_size = len(binary_data)
        add_log_message(f"Sending a total of {total_size} bytes to STM32")
        
        # Update progress bar to show processing
        progress_var.set(10)
        root.update_idletasks()
        
        # Send a single 'h' command to indicate we're sending hex data
        add_log_message("Sending 'h' command to initiate data transfer")
        ser.write(b"h\r\n")
        
        # Wait for STM32 to prepare for receiving
        time.sleep(0.5)
        
        # Send the total size information first (as a 4-byte integer, little-endian)
        size_bytes = total_size.to_bytes(4, byteorder='little')
        ser.write(size_bytes)
        add_log_message(f"Sent size information: {total_size} bytes")
        
        # Wait for STM32 to process size information
        time.sleep(0.2)
        
        
        # Send the entire binary data in one operation
        add_log_message(f"Sending {total_size} bytes of binary data...")
        progress_var.set(20)
        root.update_idletasks()
        
        # Write the entire binary data at once
        ser.write(binary_data)
        
        # Update progress after sending
        progress_var.set(90)
        root.update_idletasks()
        add_log_message(f"Sent {total_size} bytes in one operation")
        
        # Send end command to finalize programming
        add_log_message("Sending 'e' command to start Arduino programming")
        time.sleep(0.5)  # Short delay before sending end command
        ser.write(b"e\r\n")
        
        # Wait for programming to complete 
        add_log_message("Waiting for STM32 to program Arduino (10 seconds)...")
        
        # Show progress during waiting period
        for i in range(10):
            progress_var.set(90 + i)
            root.update_idletasks()
            time.sleep(1.0)
        
        # Final progress update
        progress_var.set(100)
        root.update_idletasks()
        
        # Assume success after waiting
        add_log_message("Programming completed (based on timeout)")
        messagebox.showinfo("Programming Complete", 
                           "Programming operation completed. Please verify the device is functioning as expected.")
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to send hex data:\n{e}")
        add_log_message(f"Error sending hex data: {str(e)}")

def process_hex_to_binary():
    """Convert HEX file format to binary data for the STM32"""
    global parsed_lines
    
    # Constants for memory size
    MAX_FLASH_ADDRESS = 0x8000  # 32KB flash memory
    
    binary_data = bytearray(MAX_FLASH_ADDRESS)  # Create a buffer of MAX_FLASH_ADDRESS size
    for i in range(MAX_FLASH_ADDRESS):
        binary_data[i] = 0xFF  # Initialize with 0xFF (erased flash)
    
    for line in parsed_lines:
        if not line.startswith(':'):
            continue
            
        # Parse the HEX line
        byte_count = int(line[1:3], 16)
        address = int(line[3:7], 16)
        record_type = int(line[7:9], 16)
        
        # Only process data records (type 00)
        if record_type == 0:
            data = line[9:9+byte_count*2]
            
            # Convert hex string to binary and place at the correct address
            for i in range(0, len(data), 2):
                if address + i//2 < len(binary_data):
                    binary_data[address + i//2] = int(data[i:i+2], 16)
    
    # Find the last non-0xFF byte to determine the actual used size
    actual_size = len(binary_data)
    for i in range(len(binary_data)-1, -1, -1):
        if binary_data[i] != 0xFF:
            actual_size = i + 1
            break
    
    return binary_data[:actual_size]

def enter_passthrough_mode():
    """Enter passthrough mode for direct serial communication"""
    global ser, passthrough_active
    
    if ser is None or not ser.is_open:
        messagebox.showerror("Error", "UART connection not established!")
        add_log_message("Error: No connection for passthrough mode")
        return
    
    try:
        # Send the passthrough command
        ser.write(b"c\n")
        
        # Show a waiting message while the STM processes the command
        add_log_message("Sending passthrough mode command...")
        
        # Wait a moment for the STM to switch to passthrough mode
        time.sleep(0.5)
        
        # Set the passthrough mode flag to true
        passthrough_active = True
        
        # Update UI to show passthrough mode is active
        btn_passthrough.config(text="Serial Active", bg=COLORS['label_fg'])
        btn_exit_passthrough.config(state='normal')
        update_mode_indicator()
        
        add_log_message("Entered passthrough mode - messages now appearing in Serial Monitor")
        
        # Switch to Serial Monitor tab
        show_monitor()
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to enter passthrough mode:\n{e}")
        add_log_message(f"Error entering passthrough mode: {str(e)}")

def exit_passthrough_mode():
    """Exit passthrough mode"""
    global ser, passthrough_active
    
    if ser is None or not ser.is_open:
        messagebox.showerror("Error", "UART connection not established!")
        add_log_message("Error: No connection for exiting passthrough mode")
        return
    
    if not passthrough_active:
        add_log_message("Not in passthrough mode")
        return
    
    try:
        # Send the exit command
        ser.write(b"x\n")
        
        # Wait a moment for the STM to process the exit command
        add_log_message("Sending exit Serial command...")
        time.sleep(0.5)
        
        # Set passthrough mode inactive
        passthrough_active = False
        
        # Reset UI to show passthrough mode is inactive
        btn_passthrough.config(text="Serial Mode", bg=COLORS['button_bg'])
        btn_exit_passthrough.config(state='disabled')
        update_mode_indicator()
        
        # Switch to logs view
        show_logs()
        
        add_log_message("Exited passthrough mode - messages now appearing in Activity Log")
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to exit Serial mode:\n{e}")
        add_log_message(f"Error exiting passthrough mode: {str(e)}")

def send_serial_command():
    """Send a custom serial command in passthrough mode"""
    global ser
    
    if ser is None or not ser.is_open:
        messagebox.showerror("Error", "UART connection not established!")
        add_log_message("Error: No connection for serial command")
        return
    
    command = serial_command_entry.get()
    if not command:
        return
        
    try:
        # Send the command with newline
        command_with_newline = command + "\n"
        ser.write(command_with_newline.encode())
        add_log_message(f"Sent: {command}")
        
        # Clear the entry
        serial_command_entry.delete(0, tk.END)
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to send serial command:\n{e}")
        add_log_message(f"Error sending serial command: {str(e)}")

def add_log_message(message):
    """Add a message to the log with timestamp"""
    timestamp = time.strftime("%H:%M:%S", time.localtime())
    log_text.config(state=tk.NORMAL)
    log_text.insert(tk.END, f"[{timestamp}] {message}\n")
    log_text.see(tk.END)
    log_text.config(state=tk.DISABLED)

def clear_log():
    """Clear the log text area"""
    log_text.config(state=tk.NORMAL)
    log_text.delete(1.0, tk.END)
    log_text.config(state=tk.DISABLED)

def pop_out_logs():
    """Open log view in a new window"""
    create_detachable_window("Program Logs", log_frame)

def pop_out_monitor():
    """Open serial monitor in a new window"""
    create_detachable_window("Serial Monitor", serial_monitor_frame)

def pop_out_plot():
    """Open plot view in a new window"""
    create_detachable_window("Serial Data Plot", plot_frame)

def show_logs():
    """Switch to logs view"""
    global current_view
    current_view = "logs"
    
    # Hide all containers first
    monitor_container.pack_forget()
    plot_container.pack_forget()
    code_container.pack_forget()
    
    # Show logs container
    log_container.pack(fill="both", expand=True)
    
    # Update tab buttons
    update_tab_buttons()

def show_monitor():
    """Switch to serial monitor view"""
    global current_view
    current_view = "monitor"
    
    # Hide all containers first
    log_container.pack_forget()
    plot_container.pack_forget()
    code_container.pack_forget()
    
    # Show monitor container
    monitor_container.pack(fill="both", expand=True)
    
    # Update tab buttons
    update_tab_buttons()

def show_plot():
    """Switch to plot view"""
    global current_view
    current_view = "plot"
    
    # Hide all containers first
    log_container.pack_forget()
    monitor_container.pack_forget()
    code_container.pack_forget()
    
    # Show plot container
    plot_container.pack(fill="both", expand=True)
    
    # Update tab buttons
    update_tab_buttons()

def show_editor():
    """Switch to code editor view"""
    global current_view
    current_view = "editor"
    
    # Hide all containers first
    log_container.pack_forget()
    monitor_container.pack_forget()
    plot_container.pack_forget()
    
    # Show editor container
    code_container.pack(fill="both", expand=True)
    
    # Update tab buttons
    update_tab_buttons()

def update_mode_indicator():
    """Update the passthrough mode indicator"""
    if passthrough_active:
        passthrough_indicator.configure(style='Connected.TLabel')
        passthrough_indicator.config(text="Mode: Passthrough")
    else:
        passthrough_indicator.configure(style='Custom.TLabel')
        passthrough_indicator.config(text="Mode: Normal")

# Create simple code editor without pop-out button
def create_code_editor():
    """Create and return a code editor frame without pop-out button"""
    editor_frame = ttk.LabelFrame(code_container, text="Arduino Code Editor", style='Custom.TLabelframe')
    editor_frame.pack(fill="both", expand=True, pady=(0, 10))
    
    # Create a text editor
    editor = scrolledtext.ScrolledText(editor_frame, bg=COLORS['secondary_bg'], fg=COLORS['text'], 
                                     height=30, font=('Consolas', 10))
    editor.pack(fill='both', expand=True, padx=10, pady=10)
    
    # Control buttons
    editor_controls = ttk.Frame(editor_frame, style='Custom.TFrame')
    editor_controls.pack(fill='x', padx=10, pady=(0, 10))
    
    # Button to save code to a file
    btn_save_code = create_button(editor_controls, "Save Code", lambda: save_code(editor.get(1.0, tk.END)))
    btn_save_code.pack(side='left', padx=5)
    
    # Button to load code from a file
    btn_load_code = create_button(editor_controls, "Load Code", lambda: load_code(editor))
    btn_load_code.pack(side='left', padx=5)
    
    # Button to compile code
    btn_compile = create_button(editor_controls, "Compile", lambda: compile_code(editor.get(1.0, tk.END)))
    btn_compile.pack(side='left', padx=5)
    
    return editor_frame

# Create main application window
root = tk.Tk()
root.title("Wireless Arduino Programmer")
root.geometry("1200x800")
root.configure(bg=COLORS['background'])

# Create custom styles
create_custom_style()

# Main container with two sections
main_container = ttk.Frame(root, style='Custom.TFrame')
main_container.pack(fill='both', expand=True, padx=20, pady=20)

# Status bar at top
status_bar = ttk.Frame(main_container, style='Custom.TFrame')
status_bar.pack(fill='x', pady=(0, 10))

# Add passthrough mode indicator
passthrough_indicator = ttk.Label(status_bar, text="Mode: Normal", style='Custom.TLabel')
passthrough_indicator.pack(side='left', padx=5)

# Add signature verification indicator
verify_indicator = ttk.Label(status_bar, text="Signature: Not Verified", style='Disconnected.TLabel')
verify_indicator.pack(side='right', padx=5)

# Top section for tab buttons
tabs_frame = ttk.Frame(main_container, style='Custom.TFrame')
tabs_frame.pack(fill='x', pady=(0, 10))

# Tab buttons
btn_logs = create_tab_button(tabs_frame, "Program Logs", show_logs, is_active=True)
btn_logs.pack(side='left', padx=5)

# Add this after your existing tab button declarations:
btn_editor = create_tab_button(tabs_frame, "Arduino Code Editor", show_editor)
btn_editor.pack(side='left', padx=5)

btn_monitor = create_tab_button(tabs_frame, "Serial Monitor", show_monitor)
btn_monitor.pack(side='left', padx=5)

btn_plot = create_tab_button(tabs_frame, "Serial Data Plotter", show_plot)
btn_plot.pack(side='left', padx=5)

# Content section divided into left and right columns
content_container = ttk.Frame(main_container, style='Custom.TFrame')
content_container.pack(fill='both', expand=True)

# Left column for controls - FIXED: removed width parameter
left_column = ttk.Frame(content_container, style='Custom.TFrame')
left_column.pack(side='left', fill='both', expand=False, padx=(0, 10))

# Set a minimum width for left_column using a workaround
left_column_sizer = ttk.Frame(left_column, width=400, height=1)
left_column_sizer.pack(side='top')

# Right column for views (logs, monitor, plot)
right_column = ttk.Frame(content_container, style='Custom.TFrame')
right_column.pack(side='right', fill='both', expand=True, padx=(10, 0))

code_container = ttk.Frame(right_column, style='Custom.TFrame')

# Container frames for each view
log_container = ttk.Frame(right_column, style='Custom.TFrame')
monitor_container = ttk.Frame(right_column, style='Custom.TFrame')
plot_container = ttk.Frame(right_column, style='Custom.TFrame')

# Board Selection Frame
board_frame = ttk.LabelFrame(left_column, text="Board Selection", style='Custom.TLabelframe')
board_frame.pack(fill="x", pady=(0, 10))

board_var = tk.StringVar()
board_dropdown = ttk.Combobox(board_frame, textvariable=board_var, 
                             values=list(BOARD_SIGNATURES.keys()),
                             style='Custom.TCombobox')
board_dropdown.pack(side='left', padx=10, pady=10)

btn_select_board = create_button(board_frame, "Select Board", select_board)
btn_select_board.pack(side='left', padx=10, pady=10)

# File Operations Frame
file_frame = ttk.LabelFrame(left_column, text="File Operations", style='Custom.TLabelframe')
file_frame.pack(fill="x", pady=(0, 10))

btn_load = create_button(file_frame, "Load HEX File", load_hex_file)
btn_load.pack(side='left', padx=10, pady=10)

label_loaded_file = ttk.Label(file_frame, text="Loaded File: None",
                             style='Custom.TLabel')
label_loaded_file.pack(side='left', padx=10, pady=10)

# Create a Treeview widget with custom style for HEX file display
tree_frame = ttk.Frame(left_column, style='Custom.TFrame')
tree_frame.pack(fill='both', expand=True, pady=(0, 10))

columns = ("Byte Count", "Address", "Record Type", "Data", "Checksum")
tree = ttk.Treeview(tree_frame, columns=columns, show="headings",
                    style='Custom.Treeview', height=8)  # Reduced height

# Configure the treeview
for col in columns:
    tree.heading(col, text=col)
    tree.column(col, anchor="center", width=150)

# Fix empty Treeview background
tree.tag_configure('empty', background=COLORS['treeview_bg'])

# Add scrollbars
vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=tree.yview)
hsb = ttk.Scrollbar(tree_frame, orient="horizontal", command=tree.xview)
tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)

# Grid the treeview and scrollbars
tree.grid(column=0, row=0, sticky='nsew')
vsb.grid(column=1, row=0, sticky='ns')
hsb.grid(column=0, row=1, sticky='ew')
tree_frame.grid_columnconfigure(0, weight=1)
tree_frame.grid_rowconfigure(0, weight=1)

# UART Settings Frame - Single port for TX and RX
uart_frame = ttk.LabelFrame(left_column, text="UART Settings", style='Custom.TLabelframe')
uart_frame.pack(fill="x", pady=(0, 10))

# Baud rate settings
baud_frame = ttk.Frame(uart_frame, style='Custom.TFrame')
baud_frame.pack(fill='x', padx=10, pady=(10, 5))

baud_label = ttk.Label(baud_frame, text="Baud Rate:", style='Custom.TLabel')
baud_label.pack(side='left', padx=5)

baud_var = tk.StringVar(value="115200")  # Default to 115200 for HC-05
baud_dropdown = ttk.Combobox(baud_frame, textvariable=baud_var,
                            values=["9600", "38400", "57600", "115200"],
                            width=10, style='Custom.TCombobox')
baud_dropdown.pack(side='left', padx=5)

btn_refresh = create_button(baud_frame, "Refresh Ports", detect_ports)
btn_refresh.pack(side='right', padx=5)

# Port selection
port_frame = ttk.Frame(uart_frame, style='Custom.TFrame')
port_frame.pack(fill='x', padx=10, pady=5)

port_label = ttk.Label(port_frame, text="COM Port:", style='Custom.TLabel')
port_label.pack(side='left', padx=5)

com_var = tk.StringVar()
com_dropdown = ttk.Combobox(port_frame, textvariable=com_var,
                           width=30, style='Custom.TCombobox')
com_dropdown.pack(side='left', padx=5)

status_label = ttk.Label(port_frame, text="Disconnected", style='Disconnected.TLabel')
status_label.pack(side='right', padx=5)

# Connection buttons
conn_buttons_frame = ttk.Frame(uart_frame, style='Custom.TFrame')
conn_buttons_frame.pack(fill='x', padx=10, pady=5)

btn_connect = create_button(conn_buttons_frame, "Connect", connect_serial)
btn_connect.pack(side='left', padx=5)

btn_disconnect = create_button(conn_buttons_frame, "Disconnect", disconnect_serial)
btn_disconnect.pack(side='left', padx=5)

# Programming Controls Frame
prog_frame = ttk.LabelFrame(left_column, text="Programming Controls", style='Custom.TLabelframe')
prog_frame.pack(fill="x", pady=(0, 10))

btn_check_signature = create_button(prog_frame, "Check Signature", check_signature)
btn_check_signature.pack(side='left', padx=10, pady=10)

btn_send = create_button(prog_frame, "Program Device", send_hex_data)
btn_send.pack(side='left', padx=10, pady=10)

btn_passthrough = create_button(prog_frame, "Serial Mode", enter_passthrough_mode)
btn_passthrough.pack(side='left', padx=10, pady=10)

btn_exit_passthrough = create_button(prog_frame, "Exit Serial", exit_passthrough_mode, state='disabled')
btn_exit_passthrough.pack(side='left', padx=10, pady=10)

# Progress bar for programming
progress_frame = ttk.Frame(left_column, style='Custom.TFrame')
progress_frame.pack(fill='x', pady=(0, 10))

progress_var = tk.IntVar(value=0)
progress_bar = ttk.Progressbar(progress_frame, variable=progress_var, maximum=100)
progress_bar.pack(fill='x', padx=10, pady=5)

progress_label = ttk.Label(progress_frame, text="Programming Progress", style='Custom.TLabel')
progress_label.pack(pady=5)

# Log Frame - Will go in log_container
log_frame = ttk.LabelFrame(log_container, text="Activity Log", style='Custom.TLabelframe')
log_frame.pack(fill="both", expand=True, pady=(0, 10))

log_text = scrolledtext.ScrolledText(log_frame, bg=COLORS['secondary_bg'], fg=COLORS['text'], height=20)
log_text.pack(fill='both', expand=True, padx=10, pady=10)
log_text.config(state=tk.DISABLED)  # Make it read-only

log_controls = ttk.Frame(log_frame, style='Custom.TFrame')
log_controls.pack(fill='x', padx=10, pady=(0, 10))

btn_clear_log = create_button(log_controls, "Clear Log", clear_log)
btn_clear_log.pack(side='left', padx=5)

#btn_pop_out_log = create_button(log_controls, "Pop Out", lambda: pop_out_logs())
#btn_pop_out_log.pack(side='left', padx=5)

# Serial Monitor Frame - Will go in monitor_container
serial_monitor_frame = ttk.LabelFrame(monitor_container, text="Serial Monitor", style='Custom.TLabelframe')
serial_monitor_frame.pack(fill="both", expand=True, pady=(0, 10))

serial_monitor = scrolledtext.ScrolledText(serial_monitor_frame, bg=COLORS['secondary_bg'], fg=COLORS['text'], 
                                         height=20, font=('Consolas', 10))
serial_monitor.pack(fill='both', expand=True, padx=10, pady=10)
serial_monitor.config(state=tk.DISABLED)  # Make it read-only

serial_monitor_controls = ttk.Frame(serial_monitor_frame, style='Custom.TFrame')
serial_monitor_controls.pack(fill='x', padx=10, pady=(0, 10))

serial_command_entry = ttk.Entry(serial_monitor_controls, width=50)
serial_command_entry.pack(side='left', fill='x', expand=True, padx=5)

btn_send_command = create_button(serial_monitor_controls, "Send", send_serial_command)
btn_send_command.pack(side='left', padx=5)

btn_clear_monitor = create_button(serial_monitor_controls, "Clear", clear_serial_monitor)
btn_clear_monitor.pack(side='left', padx=5)



# Plot Frame - Will go in plot_container
plot_frame = ttk.LabelFrame(plot_container, text="Serial Data Plot", style='Custom.TLabelframe')
plot_frame.pack(fill="both", expand=True, pady=(0, 10))

# Create a matplotlib figure
fig = plt.Figure(figsize=(5, 4), dpi=100)
ax = fig.add_subplot(111)
ax.set_title('Serial Data Plot')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Value')
ax.grid(True)

# Create a canvas for the figure
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.draw()
canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=10)

plot_controls = ttk.Frame(plot_frame, style='Custom.TFrame')
plot_controls.pack(fill='x', padx=10, pady=(0, 10))

btn_toggle_plot = create_button(plot_controls, "Start Plotting", toggle_plotting)
btn_toggle_plot.pack(side='left', padx=5)


# Initialize ports
detect_ports()

# Show logs view by default (already active)
show_logs()

# Initial log message
add_log_message("Application started")
add_log_message("Note: Using a single port for both transmission and receiving")
add_log_message("In passthrough mode, avoid sending inputs starting with 'p', 's', 'h', or 'e'")
add_log_message("Arduino code editor available - requires Arduino CLI to be installed")
if check_arduino_cli():
    add_log_message("Arduino CLI detected - code compilation available")
else:
    add_log_message("Arduino CLI not found - please install to enable code compilation")
editor_frame = create_code_editor()

# Start the main loop
root.mainloop()