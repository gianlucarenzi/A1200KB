import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

# --- Scancode Data (as compiled in the previous step) ---
HID_SCANCODES = {
    "No Event": 0x00,
    "ErrorRollOver": 0x01,
    "POSTFail": 0x02,
    "ErrorUndefined": 0x03,
    "A": 0x04, "B": 0x05, "C": 0x06, "D": 0x07, "E": 0x08, "F": 0x09, "G": 0x0A, "H": 0x0B,
    "I": 0x0C, "J": 0x0D, "K": 0x0E, "L": 0x0F, "M": 0x10, "N": 0x11, "O": 0x12, "P": 0x13,
    "Q": 0x14, "R": 0x15, "S": 0x16, "T": 0x17, "U": 0x18, "V": 0x19, "W": 0x1A, "X": 0x1B,
    "Y": 0x1C, "Z": 0x1D,
    "1 (!|\)": 0x1E, "2 (@|\")": 0x1F, "3 (#|£)": 0x20, "4 ($|€)": 0x21, "5 (%|,)": 0x22,
    "6 (^|.)": 0x23, "7 (&|/)": 0x24, "8 (*|;)": 0x25, "9 (:|')": 0x26, "0 (|')": 0x27,
    "Return (Enter)": 0x28, "Escape": 0x29, "Backspace": 0x2A, "Tab": 0x2B, "Spacebar": 0x2C,
    "Minus (_|-)": 0x2D, "Equals (+|=)": 0x2E, "LeftBracket ({|[)": 0x2F, "RightBracket (}|])": 0x30,
    "Backslash (||\\)": 0x31, "Non-US # (~)": 0x32, "Semicolon (:|;)": 0x33, "Quote ('|\"')": 0x34,
    "Grave Accent (~|`)": 0x35, "Comma (<|,)": 0x36, "Period (>|.)": 0x37, "Slash (?|/)": 0x38,
    "CapsLock": 0x39,
    "F1": 0x3A, "F2": 0x3B, "F3": 0x3C, "F4": 0x3D, "F5": 0x3E, "F6": 0x3F,
    "F7": 0x40, "F8": 0x41, "F9": 0x42, "F10": 0x43, "F11": 0x44, "F12": 0x45,
    "PrintScreen": 0x46, "ScrollLock": 0x47, "Pause": 0x48, "Insert": 0x49, "Home": 0x4A,
    "PageUp": 0x4B, "Delete": 0x4C, "End": 0x4D, "PageDown": 0x4E,
    "RightArrow": 0x4F, "LeftArrow": 0x50, "DownArrow": 0x51, "UpArrow": 0x52,
    "NumLock": 0x53, "Keypad /": 0x54, "Keypad *": 0x55, "Keypad -": 0x56, "Keypad +": 0x57,
    "Keypad Enter": 0x58, "Keypad 1 (End)": 0x59, "Keypad 2 (Down)": 0x5A, "Keypad 3 (PageDown)": 0x5B,
    "Keypad 4 (Left)": 0x5C, "Keypad 5": 0x5D, "Keypad 6 (Right)": 0x5E, "Keypad 7 (Home)": 0x5F,
    "Keypad 8 (Up)": 0x60, "Keypad 9 (PageUp)": 0x61, "Keypad 0 (Insert)": 0x62, "Keypad . (Delete)": 0x63,
    "Non-US \\ (|)": 0x64, "Application": 0x65, "Power": 0x66, "Keypad =": 0x67,
    "F13": 0x68, "F14": 0x69, "F15": 0x6A, "F16": 0x6B, "F17": 0x6C, "F18": 0x6D,
    "F19": 0x6E, "F20": 0x6F, "F21": 0x70, "F22": 0x71, "F23": 0x72, "F24": 0x73,
    "Execute": 0x74, "Help": 0x75, "Menu": 0x76, "Select": 0x77, "Stop": 0x78, "Again": 0x79,
    "Undo": 0x7A, "Cut": 0x7B, "Copy": 0x7C, "Paste": 0x7D, "Find": 0x7E, "Mute": 0x7F,
    "VolumeUp": 0x80, "VolumeDown": 0x81, "Locking CapsLock": 0x82, "Locking NumLock": 0x83,
    "Locking ScrollLock": 0x84, "Keypad ,": 0x85, "EqualSign": 0x86, "International1": 0x87,
    "International2": 0x88, "International3": 0x89, "International4": 0x8A, "International5": 0x8B,
    "International6": 0x8C, "International7": 0x8D, "International8": 0x8E, "International9": 0x8F,
    "Lang1": 0x90, "Lang2": 0x91, "Lang3": 0x92, "Lang4": 0x93, "Lang5": 0x94, "Lang6": 0x95,
    "Lang7": 0x96, "Lang8": 0x97, "Lang9": 0x98, "AlternateErase": 0x99, "SysReq/Attention": 0x9A,
    "Cancel": 0x9B, "Clear": 0x9C, "Prior": 0x9D, "Return": 0x9E, "Separator": 0x9F, "Out": 0xA0,
    "Oper": 0xA1, "Clear/Again": 0xA2, "CrSel/Props": 0xA3, "ExSel": 0xA4,

    # Modifiers
    "LeftControl": 0xE0, "LeftShift": 0xE1, "LeftAlt": 0xE2, "Left GUI": 0xE3,
    "RightControl": 0xE4, "RightShift": 0xE5, "RightAlt": 0xE6, "Right GUI": 0xE7
}

# Invert the dictionary for easy lookup of name by value
SCANCODES_BY_VALUE = {v: k for k, v in HID_SCANCODES.items()}
# List of scancode names for dropdowns, sorted alphabetically
SCANCODE_NAMES = sorted(HID_SCANCODES.keys())

class KeyboardMatrixConfigurator:
    def __init__(self, master):
        self.master = master
        master.title("Amiga USB Keyboard Matrix Configurator")

        self.rows_var = tk.StringVar(value="6")
        self.cols_var = tk.StringVar(value="22")
        self.matrix_data = [] # Stores selected scancodes for each cell

        # --- Input Frame ---
        input_frame = ttk.LabelFrame(master, text="Matrix Dimensions")
        input_frame.pack(pady=10, padx=10, fill="x")

        ttk.Label(input_frame, text="Rows:").grid(row=0, column=0, padx=5, pady=5)
        ttk.Entry(input_frame, textvariable=self.rows_var, width=5).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="Columns:").grid(row=0, column=2, padx=5, pady=5)
        ttk.Entry(input_frame, textvariable=self.cols_var, width=5).grid(row=0, column=3, padx=5, pady=5)

        ttk.Button(input_frame, text="Create/Update Matrix", command=self.create_matrix_grid).grid(row=0, column=4, padx=10, pady=5)

        # --- Matrix Frame ---
        self.matrix_frame = ttk.LabelFrame(master, text="Keyboard Matrix")
        self.matrix_frame.pack(pady=10, padx=10, expand=True, fill="both")

        # --- Scancode Output Frame ---
        output_frame = ttk.LabelFrame(master, text="Generated scancode_lut C Array")
        output_frame.pack(pady=10, padx=10, fill="both", expand=True)

        self.output_text = scrolledtext.ScrolledText(output_frame, width=80, height=15, wrap=tk.WORD)
        self.output_text.pack(pady=5, padx=5, expand=True, fill="both")

        # --- Generate Button ---
        ttk.Button(master, text="Generate C Array", command=self.generate_c_array).pack(pady=10)

        # Initialize an empty matrix grid on startup
        self.create_matrix_grid()

    def create_matrix_grid(self):
        # Clear existing widgets in matrix_frame
        for widget in self.matrix_frame.winfo_children():
            widget.destroy()

        try:
            rows = int(self.rows_var.get())
            cols = int(self.cols_var.get())
            if not (1 <= rows <= 100 and 1 <= cols <= 100):
                raise ValueError("Rows and Columns must be between 1 and 100.")
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))
            return

        # Initialize matrix_data with default "No Event" scancode (0x00)
        # Retain existing values if resizing to smaller dimensions
        old_matrix_data = [row[:] for row in self.matrix_data] # Deep copy
        self.matrix_data = [[0x00 for _ in range(cols)] for _ in range(rows)]

        for r in range(min(rows, len(old_matrix_data))):
            for c in range(min(cols, len(old_matrix_data[0]) if old_matrix_data else 0)):
                self.matrix_data[r][c] = old_matrix_data[r][c]

        # Create new grid
        self.matrix_buttons = []
        for r in range(rows):
            row_buttons = []
            for c in range(cols):
                scancode_value = self.matrix_data[r][c]
                scancode_name = SCANCODES_BY_VALUE.get(scancode_value, "UNKNOWN")
                btn = ttk.Button(self.matrix_frame, text=scancode_name,
                                 command=lambda r=r, c=c: self.open_scancode_selector(r, c))
                btn.grid(row=r, column=c, padx=1, pady=1, sticky="nsew")
                row_buttons.append(btn)
            self.matrix_buttons.append(row_buttons)

        # Configure row and column weights so they expand nicely
        for r in range(rows):
            self.matrix_frame.grid_rowconfigure(r, weight=1)
        for c in range(cols):
            self.matrix_frame.grid_columnconfigure(c, weight=1)

    def open_scancode_selector(self, r, c):
        selector_window = tk.Toplevel(self.master)
        selector_window.title(f"Select Scancode for ({r}, {c})")
        selector_window.transient(self.master) # Make it appear on top of the main window
        selector_window.grab_set() # Disable interaction with the main window

        current_scancode_val = self.matrix_data[r][c]
        current_scancode_name = SCANCODES_BY_VALUE.get(current_scancode_val, "No Event")
        
        selected_scancode_name = tk.StringVar(value=current_scancode_name)

        ttk.Label(selector_window, text="Select Scancode:").pack(padx=10, pady=5)
        
        # Use Combobox for scancode selection
        scancode_combobox = ttk.Combobox(selector_window, textvariable=selected_scancode_name,
                                         values=SCANCODE_NAMES, state="readonly")
        scancode_combobox.pack(padx=10, pady=5, fill="x")
        scancode_combobox.set(current_scancode_name) # Set initial value

        def save_selection():
            new_name = selected_scancode_name.get()
            new_value = HID_SCANCODES.get(new_name)
            if new_value is not None:
                self.matrix_data[r][c] = new_value
                self.matrix_buttons[r][c].config(text=new_name)
                selector_window.destroy()
            else:
                messagebox.showerror("Error", "Invalid scancode selected.")

        ttk.Button(selector_window, text="Save", command=save_selection).pack(pady=10)
        ttk.Button(selector_window, text="Cancel", command=selector_window.destroy).pack(pady=5)

    def generate_c_array(self):
        rows = int(self.rows_var.get())
        cols = int(self.cols_var.get())

        c_array_str = f"const uint8_t scancode_lut[{rows}][{cols}] = {{\n"
        for r_idx, row in enumerate(self.matrix_data):
            c_array_str += f"// R{r_idx}\n"
            c_array_str += "  {"
            for c_idx, scancode_val in enumerate(row):
                c_array_str += f"0x{scancode_val:02X}"
                if c_idx < cols - 1:
                    c_array_str += ","
                c_array_str += " "
            c_array_str += "},\n"
        c_array_str += "};"

        self.output_text.delete(1.0, tk.END)
        self.output_text.insert(tk.END, c_array_str)

# --- Main Application ---
if __name__ == "__main__":
    root = tk.Tk()
    app = KeyboardMatrixConfigurator(root)
    root.mainloop()
