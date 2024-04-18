from datetime import datetime
from time import sleep


while True:
    current_time = datetime.now()

    print("The curren t time is:", current_time.strftime("%H:%M:%S"))
    sleep(1)