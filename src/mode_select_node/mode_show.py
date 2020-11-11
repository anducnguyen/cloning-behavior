from tkinter import *

window = Tk()

window.title("Mode Switching")

lbl = Label(window, text="Mode 2\nCollision Avoidance", font=("Arial Bold", 15), bg="red", fg="black")
lbl.grid(column=1, row=1,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 5\nLong", font=("Arial Bold", 15), bg="green", fg="black")
lbl.grid(column=1, row=0,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 3\nShort", font=("Arial Bold", 15), bg="purple", fg="black")
lbl.grid(column=0, row=1,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 1\nMid_A", font=("Arial Bold", 15), bg="yellow", fg="black")
lbl.grid(column=2, row=1,padx=5, pady=5, sticky="nsew")
lbl = Label(window, text="Mode 4\nMid_F", font=("Arial Bold", 15), bg="blue", fg="black")
lbl.grid(column=1, row=2,padx=5, pady=5, sticky="nsew")

window.geometry('400x200')

window.mainloop()
x = 1
def main():
if x == 1
lbl = Label(window, text="Mode 4\nMid_F", font=("Arial Bold", 15), bg="black", fg="white")
else
lbl = Label(window, text="Mode 1\nMid_A", font=("Arial Bold", 15), bg="black", fg="white")

if __name__ == "__main__":
    main()