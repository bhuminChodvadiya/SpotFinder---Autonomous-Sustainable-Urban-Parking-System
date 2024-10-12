import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

class OptionSelectorNode(Node):

    def __init__(self):
        super().__init__('option_selector_node')
        self.get_logger().info('Option Selector Node Initialized')

        self.publisher_ = self.create_publisher(String, 'selected_option', 10)

    def publish_selected_option(self, selected_option):
        self.publisher_.publish(String(data=selected_option))

class OptionSelectorApp:

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Option Selector')
        self.root.geometry('800x850')  # Set window size

        self.style = ttk.Style()
        self.style.configure('TFrame', background='#e1d8b9')
        self.style.configure('TButton', background='#a1dbcd', font=('Arial', 12))
        self.style.configure('TRadiobutton', background='#e1d8b9', font=('Arial', 12))

        # Load and resize the map image
        map_image = Image.open("/home/af/ros2_ws/src/sf_master/src/sf_ui/resource/kronach_map.jpg")
        map_image = map_image.resize((900, 500), Image.NEAREST)  # Resize the map image
        map_photo = ImageTk.PhotoImage(map_image)
        map_label = ttk.Label(self.root, image=map_photo)
        map_label.image = map_photo  # Keep a reference to prevent garbage collection
        map_label.pack(pady=20)

        # Main Frame
        self.frame = ttk.Frame(self.root)
        self.frame.pack(pady=20)

        self.label = ttk.Label(self.frame, text='Select a Parking Option:', font=('Arial', 14))
        self.label.grid(row=0, column=0, columnspan=2, pady=10)

        self.options = {
            'Option 1: 450 meters away at Bahnhof station': 'your car will be parked at Bahnhof station',
            'Option 2: 600 meters away at Sparkesse bank': 'Sparkesse bank',
            'Option 3: 150 meters away at LCC campus': 'LCC campus',
            'Option 4: 300 meters away at VR bank': 'VR bank'
        }

        self.selected_option_var = tk.StringVar()

        row_num = 1
        for option_text, location in self.options.items():
            ttk.Radiobutton(self.frame, text=option_text, variable=self.selected_option_var, value=location).grid(row=row_num, column=0, sticky=tk.W, padx=20, pady=5)
            row_num += 1

        self.select_button = ttk.Button(self.root, text='Select', command=self.process_selection)
        self.select_button.pack(pady=20)

    def process_selection(self):
        selected_option = self.selected_option_var.get()

        if selected_option:
            self.node.publish_selected_option(selected_option)
            messagebox.showinfo('Selection', f'You have selected: {selected_option}')
            self.root.destroy()
        else:
            messagebox.showerror('Error', 'Please select an option')

def main():
    rclpy.init()
    node = OptionSelectorNode()

    app = OptionSelectorApp(node)
    app.root.mainloop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
