
import time

class FreqCounter():
    def __init__(self):
        self.start_time = 0
        self.items_counter = 0

    def tick(self):
        self.items_counter+=1

    def getTimeSec(self):
        if self.start_time is None:
            return 0
        return time.time() - self.start_time
    
    def getFreq(self):
        # print(f"{self.getTimeSec() = }; {self.items_counter = }")
        if self.getTimeSec() == 0:
            return 0
        freq = float(self.items_counter)/float(self.getTimeSec())
        return freq
    
    def cuontItems(self):
        return self.items_counter
    
    def start(self):
        print(f"[FreqCounter::start] Init freq counter.")
        self.items_counter = 0
        self.start_time = time.time()

    def stop(self):
        self.start_time = 0

    def __str__(self):
        return f"{self.getFreq():.2f}"