import tkinter as tk
import subprocess
import threading

def button_pressed(value):
    # This function will be used for the first 8 buttons to send a value
    subprocess.run(['python', 'goPosition.py', str(value)])

def run_file(file_path):
    # This function runs another Python file
    subprocess.run(['python', file_path])

def run_in_thread(func, *args):
    thread = threading.Thread(target=func, args=args)
    thread.start()

def create_gui():
    window = tk.Tk()
    window.title("Advanced Button GUI")
    window.geometry("500x400")  # Set the window size

    # Manually define each button
    btn1 = tk.Button(window, text="Object 1", command=lambda: button_pressed(1))
    btn1.grid(row=0, column=0, padx=10, pady=10)

    btn5 = tk.Button(window, text="Object 1 (failure 1)", command=lambda: button_pressed(5))
    btn5.grid(row=0, column=1, padx=10, pady=10)

    btn9 = tk.Button(window, text="Object 1 (failure 2)", command=lambda: button_pressed(9))
    btn9.grid(row=0, column=2, padx=10, pady=10)

    btn2 = tk.Button(window, text="Object 2", command=lambda: button_pressed(2))
    btn2.grid(row=1, column=0, padx=10, pady=10)

    btn6 = tk.Button(window, text="Object 2 (failure 1)", command=lambda: button_pressed(6))
    btn6.grid(row=1, column=1, padx=10, pady=10)

    btn10 = tk.Button(window, text="Object 2 (failure 2)", command=lambda: button_pressed(10))
    btn10.grid(row=1, column=2, padx=10, pady=10)

    btn3 = tk.Button(window, text="Object 3", command=lambda: button_pressed(3))
    btn3.grid(row=2, column=0, padx=10, pady=10)

    btn7 = tk.Button(window, text="Object 3 (failure 1)", command=lambda: button_pressed(7))
    btn7.grid(row=2, column=1, padx=10, pady=10)

    btn11 = tk.Button(window, text="Object 3 (failure 2)", command=lambda: button_pressed(11))
    btn11.grid(row=2, column=2, padx=10, pady=10)

    btn4 = tk.Button(window, text="Object 4", command=lambda: button_pressed(4))
    btn4.grid(row=3, column=0, padx=10, pady=10)

    btn8 = tk.Button(window, text="Object 4 (failure 1)", command=lambda: button_pressed(8))
    btn8.grid(row=3, column=1, padx=10, pady=10)

    btn12 = tk.Button(window, text="Object 4 (failure 2)", command=lambda: button_pressed(12))
    btn12.grid(row=3, column=2, padx=10, pady=10)

    # Buttons for running separate Python files
    btn9 = tk.Button(window, text="Get Camera", command=lambda: run_in_thread(run_file, 'get_camera.py'))
    btn9.grid(row=4, column=0, padx=10, pady=50)

    btn10 = tk.Button(window, text="Rotate Head", command=lambda: run_in_thread(run_file, 'head-right.py'))
    btn10.grid(row=4, column=1, padx=10, pady=50)
    btn11 = tk.Button(window, text="TF Function", command=lambda: run_in_thread(run_file, 'TF.py'))
    btn11.grid(row=5, column=0, padx=10, pady=0)
    btn11 = tk.Button(window, text="Ready Position", command=lambda: run_in_thread(run_file, 'ready_position.py'))
    btn11.grid(row=5, column=1, padx=10, pady=0)

    window.mainloop()

if __name__ == "__main__":
    create_gui()
