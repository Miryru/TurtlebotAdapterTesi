import socket
import tkinter as tk
from tkinter import messagebox
import argparse


class HumanAdapter:
    def __init__( self, IP, port):
    
        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind and listen
        self.serverSocket.bind((IP,port))
        self.serverSocket.listen(10)
        print ("Listening on %s:%s"%(IP, port))
        
    def run(self):
        while True:
            
            (clientConnected, clientAddress) = self.serverSocket.accept()
            print ("Accepted a connection request from %s:%s"%(clientAddress[0], clientAddress[1]))
            dataFromClient = clientConnected.recv(1024)
            print (dataFromClient)
            request = str(dataFromClient.decode())
            ID = request.split("|")[0]
            request_type = request.split("|")[1]
            print(request_type)
            data = request.split("|")[2]
            #print(data)
            
            self.window = tk.Tk()
            # Set the window title
            self.window.title("Task Manager")
            # Set the window size
            self.window.geometry("500x500")
            self.window.configure(bg='white', border=10,)

            
            if request_type=='DECLARE':
                clientConnected.send("ACK")
                print("ACK")
            elif request_type == "PERFORMING":
                print ('performing')
                try:
                    clientConnected.send(self.performAction(data))
                except:
                    clientConnected.send(str(False).encode())
                
            elif request_type == "VERIFYING":
                print("verifying")
                perceived_outcome = self.performVerification(data)
                clientConnected.send(perceived_outcome)
                    
    def performAction(self ,data):
        action_ID = data.split("/")[1]
        self.action_completed = False
        
        if action_ID == 'A1':
            self.label = tk.Label(self.window, text="Please take the package.", font=("Arial", 20), bg='white')
            self.label.pack()   

        elif action_ID == 'A2':
            self.label = tk.Label(self.window, text="Please bring the package.", font=("Arial", 20) , bg='white')
            self.label.pack()

           
            
        self.label = tk.Label(self.window, text="Did you complete the task?", font=("Arial", 20), bg='white')
        self.label.pack()
        self.finished_button = tk.Button(self.window, text="Task Finished", command=self.task_finished, font=("Arial", 20))
        self.finished_button.pack(pady=10)

        # Create a button for when the task is not finished
        self.not_finished_button = tk.Button(self.window, text="Task Not Finished", command=self.task_not_finished, font=("Arial", 20))
        self.not_finished_button.pack(pady=10)
        
        self.window.after(440000, self.timeout)
        self.window.mainloop()
        self.window.destroy()
        if self.action_completed:
            print ('action completed')
            return (str(True).encode())
        else:
            print('action not completed')
            return (str(False).encode())
        
          
    def task_finished(self):
        messagebox.showinfo("Task Status", "Task finished!")
        self.action_completed = True
        self.window.quit()

    def task_not_finished(self):
        messagebox.showinfo("Task Status", "Task not finished!")
        self.action_completed = False
        self.window.quit()    
            
    def performVerification(self, data):
        action_ID = data.split("/")[1]
        perceived_outcome = str(False).encode()
        self.verification_completed= False
        
        if action_ID == 'A1':
        #show the same interface with 2 buttons for the human to press
            self.label = tk.Label(self.window, text="Did the robot reach the right position?", font=("Arial", 20))
            self.label.pack()
        elif action_ID == 'A2':
            self.label = tk.Label(self.window, text="Did the robot complete the task?", font=("Arial", 20))
            self.label.pack()
    
        # Create a button for when the task is finished
        self.finished_button = tk.Button(self.window, text="Yes", command=self.verify_task_finished, font=("Arial", 20))
        self.finished_button.pack(pady=10)
        
        
        # Create a button for when the task is not finished
        self.not_finished_button = tk.Button(self.window, text="No", command=self.verify_task_not_finished, font=("Arial", 20))
        self.not_finished_button.pack(pady=10)
        
        
        self.window.after(440000, self.timeout)
        self.window.mainloop()
        self.window.destroy()
        
        if self.verification_completed:
            perceived_outcome = str(True).encode()
        
        else:
            perceived_outcome = str(False).encode()
        
        return perceived_outcome
    
    def verify_task_finished(self):
        messagebox.showinfo("Task Status", "Task verified!")
        self.verification_completed = True
        self.window.quit()
    def verify_task_not_finished(self):
        messagebox.showinfo("Task Status", "Task not verified!")
        self.verification_completed = False
        self.window.quit()
    
    def timeout(self):
        self.window.quit()
        self.window.destroy()
        return (str(False).encode())
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=19000)
    args = parser.parse_args()

    robot=HumanAdapter(args.ip, args.port)
    robot.run()
if __name__ == "__main__":
    main()

    

    
