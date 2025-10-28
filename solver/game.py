import time
from logic import Logic
from utils import get_current_time_ms

CLOCK_MS = 15000


class Game:

    def __init__(self,
                 instance,
                 learning,
                 show,
                 asp,
                 horizon=1,
                 radius=1,
                 cache=1):
        self.instance = instance
        self.learning = learning
        self.asp = asp
        self.show = show
        self.cache = cache
        if show:
            from interface import Interface
            self.interface = Interface(instance=instance)
        if asp:
            self.logic = Logic(instance, learning, horizon, radius)
            
    def set_ms_clock(self, ms):
        global CLOCK_MS
        CLOCK_MS = ms

    def next_step(self):
        global CLOCK_MS
        # call this function again in 1 second
        time_pre_compute = get_current_time_ms()

        if self.instance.robot == self.instance.target:
            if self.show:
                self.interface.frame.destroy()
            return

        if not self.asp:
            action = self.learning.get_action(self.instance)
        else:
            if self.logic.ctl is None:
                self.logic.setup()
            ref_action = self.learning.get_action(self.instance)
            action = self.logic.get_action(self.cache)

            if action != ref_action:
                self.instance.interceptions += 1

        if action is None:
            if self.show:
                self.interface.frame.destroy()
            return
        print("Chosen action: ", action)
        print("send command to move..")
        t0=time.time()
        self.instance.execute(action)
        control_time = time.time()-t0
        print(f"control time: {control_time:.2f} sec")
        print("done moving.")
        if (control_time*1000) > CLOCK_MS:
            CLOCK_MS = int(control_time*1000*1.2)+1000
            #print(f"Adjusted CLOCK_MS to {CLOCK_MS} ms")
        elif (control_time*1000)/CLOCK_MS<0.7:
            CLOCK_MS = int(control_time*1000*1.2)+1000
            #print(f"Adjusted CLOCK_MS to {CLOCK_MS} ms")
        
        self.instance.emulate_obstacles()
        self.instance.check_violations()
        if self.show:
            self.interface.place_piece("robot", self.instance.robot)
            if len(self.instance.obstacles) > 0:
                self.interface.remove_pieces("obstacle")
                for c, f in enumerate(self.instance.obstacles):
                    self.interface.add_piece("obstacle_" + str(c),
                                             self.interface.obstacle_image, f)
        
        
        time_post_compute = get_current_time_ms()
        time_diff = time_post_compute - time_pre_compute

        self.instance.times.append(time_diff)

        next_step_delta = CLOCK_MS - time_diff
        if next_step_delta < 0:
            next_step_delta = 1
        if self.show:
            self.interface.after(next_step_delta, self.next_step)
        else:
            self.next_step()

    def run(self):
        self.next_step()
        if self.show:
            self.interface.mainloop()
