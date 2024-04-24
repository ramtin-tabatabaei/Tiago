import tkinter as tk
import subprocess
import threading
def button_pressed(button, puzzle_number, value):
    # This function will be used for the first 8 buttons to send a value
    # subprocess.run(['python3', 'goPosition.py', str(value), str(value)])
    button.config(bg='red')
    subprocess.run(['python', 'goPosition.py', str(value), str(puzzle_number)])

def button_pressed_home(value):
    # Append the button number to the array
    # Destroy the current window and open the advanced GUI
    home_window.destroy()
    create_gui(value)

def run_file(file_path):
    # This function runs another Python file
    subprocess.run(['python', file_path])

def run_in_thread(func, *args):
    thread = threading.Thread(target=func, args=args)
    thread.start()

def go_home():
    # Function to return to home window
    window.destroy()
    create_home_window()

def create_home_window():
    global home_window
    home_window = tk.Tk()
    home_window.title("Home Window")
    home_window.geometry("300x200")
    # Create buttons 1 to 4
    # Manually define each button
    btn1 = tk.Button(home_window, text="Puzzle 1 (Cat)", command=lambda: button_pressed_home(1))
    btn1.grid(row=0, column=0, padx=10, pady=10)
    btn2 = tk.Button(home_window, text="Puzzle 2 (Rocket)", command=lambda: button_pressed_home(2))
    btn2.grid(row=1, column=0, padx=10, pady=10)
    btn3 = tk.Button(home_window, text="Puzzle 3 (Turtle)", command=lambda: button_pressed_home(3))
    btn3.grid(row=2, column=0, padx=10, pady=10)
    btn4 = tk.Button(home_window, text="Puzzle 4 (Rabit)", command=lambda: button_pressed_home(4))
    btn4.grid(row=3, column=0, padx=10, pady=10)
    # btn5 = tk.Button(home_window, text="Start Rosbag", command=lambda: run_in_thread(run_file, 'record_rosbag.py'))
    # btn5.grid(row=0, column=1, padx=10, pady=10)

    home_window.mainloop()
def create_gui(Puzzle_number):
    global window
    window = tk.Tk()
    window.title("Advanced Button GUI")
    window.geometry("500x500")  # Set the window size
    # Manually define each button
    btn1 = tk.Button(window, text="Object 1", command=lambda: button_pressed(btn1, Puzzle_number, 1))
    btn1.grid(row=0, column=0, padx=10, pady=10)
    btn5 = tk.Button(window, text="Object 1 (failure 1)", command=lambda: button_pressed(btn5, Puzzle_number, 5))
    btn5.grid(row=0, column=1, padx=10, pady=10)
    btn9 = tk.Button(window, text="Object 1 (failure 2)", command=lambda: button_pressed(btn9, Puzzle_number, 9))
    btn9.grid(row=0, column=2, padx=10, pady=10)
    btn2 = tk.Button(window, text="Object 2", command=lambda: button_pressed(btn2, Puzzle_number, 2))
    btn2.grid(row=1, column=0, padx=10, pady=10)
    btn6 = tk.Button(window, text="Object 2 (failure 1)", command=lambda: button_pressed(btn6, Puzzle_number, 6))
    btn6.grid(row=1, column=1, padx=10, pady=10)
    btn10 = tk.Button(window, text="Object 2 (failure 2)", command=lambda: button_pressed(btn10, Puzzle_number, 10))
    btn10.grid(row=1, column=2, padx=10, pady=10)
    btn3 = tk.Button(window, text="Object 3", command=lambda: button_pressed(btn3, Puzzle_number, 3))
    btn3.grid(row=2, column=0, padx=10, pady=10)
    btn7 = tk.Button(window, text="Object 3 (failure 1)", command=lambda: button_pressed(btn7, Puzzle_number, 7))
    btn7.grid(row=2, column=1, padx=10, pady=10)
    btn11 = tk.Button(window, text="Object 3 (failure 2)", command=lambda: button_pressed(btn11, Puzzle_number, 11))
    btn11.grid(row=2, column=2, padx=10, pady=10)
    btn4 = tk.Button(window, text="Object 4", command=lambda: button_pressed(btn4, Puzzle_number, 4))
    btn4.grid(row=3, column=0, padx=10, pady=10)
    btn8 = tk.Button(window, text="Object 4 (failure 1)", command=lambda: button_pressed(btn8, Puzzle_number, 8))
    btn8.grid(row=3, column=1, padx=10, pady=10)
    btn12 = tk.Button(window, text="Object 4 (failure 2)", command=lambda: button_pressed(btn12, Puzzle_number, 12))
    btn12.grid(row=3, column=2, padx=10, pady=10)
    # Buttons for running separate Python files
    # btn13 = tk.Button(window, text="Get Camera", command=lambda: run_in_thread(run_file, 'get_camera.py'))
    # btn13.grid(row=4, column=0, padx=10, pady=50)
    btn13 = tk.Button(window, text="Rotate Head (1)", command=lambda: run_in_thread(run_file, 'head-rotate1.py'))
    btn13.grid(row=4, column=0, padx=10, pady=50)
    btn14 = tk.Button(window, text="Rotate Head (2)", command=lambda: run_in_thread(run_file, 'head-rotate2.py'))
    btn14.grid(row=4, column=1, padx=10, pady=50)
    btn15 = tk.Button(window, text="Rotate Head (3)", command=lambda: run_in_thread(run_file, 'head-rotate3.py'))
    btn15.grid(row=4, column=2, padx=10, pady=50)
    btn16 = tk.Button(window, text="Get Camera", command=lambda: run_in_thread(run_file, 'get_camera.py'))
    btn16.grid(row=5, column=0, padx=10, pady=0)
    btn17 = tk.Button(window, text="TF Function", command=lambda: run_file('TF.py'))
    btn17.grid(row=5, column=1, padx=10, pady=0)
    btn18 = tk.Button(window, text="Ready Position", command=lambda: run_in_thread(run_file, 'ready_position.py'))
    btn18.grid(row=5, column=2, padx=10, pady=0)
    # Additional 'Home' button to return to the home window
    home_btn = tk.Button(window, text="Home", command=go_home)
    home_btn.grid(row=6, column=1, padx=10, pady=30)
    window.mainloop()
if __name__ == "__main__":
    create_home_window()