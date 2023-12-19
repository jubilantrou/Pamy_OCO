import o80_pam
import o80
import time

segment_id = "real_robot"
frontend = o80_pam.FrontEnd(segment_id)

###
###
###

# Test the pressure limits of joints.

p_ago = (20000,19000,17000,17000)
p_antago = (20000,20000,17000,17000)
duration = o80.Duration_us.seconds(4)

frontend.add_command(p_ago, p_antago, duration, o80.Mode.QUEUE)
frontend.pulse()
time.sleep(5)
print("reached state robot under high pressure command: {}".format(frontend.latest()))

# frontend.add_command(p_low, p_low, duration, o80.Mode.QUEUE)
# frontend.pulse()
# time.sleep(5)
# print("reached state robot under low pressure command: {}".format(frontend.latest()))
