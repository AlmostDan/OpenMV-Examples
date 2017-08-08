# Object tracking with keypoints example.
# Show the camera an object and then run the script. A set of keypoints will be extracted
# once and then tracked in the following frames. If you want a new set of keypoints re-run
# the script. NOTE: see the docs for arguments to tune find_keypoints and match_keypoints.
import sensor, time, image, utime
from pyb import UART
from pyb import LED

# -------------------------------------------------- DEFINES

# mode of work with the cam
# "arm" - camera is connected to uArm and drives it.
# "offline" - camera is connected to computer w/o serial com to uArm. For debugging purposes.
MODE = "arm"

# print messages exchanged with the arm through the uart
PRINT_ARM_COMM = True

# camera image resolution
IMG_WIDTH = 320
IMG_HEIGHT = 240

# Camera position.  "horizontal" = pointing under the arm, top of the image is towards the base. Original.
#                   "vertical" = pointing to the front of the arm
# NOTE: Currently there is a bug in uArm due to which the arm freezes when it is at the top (z=~160+)
# and receives a differential command to move up.
CAM_POS = "horizontal"

# number of ms of one iteration (=tracked item matching, issuing command, time for the uArm to move).
ITER_TIME_MS = 300 # default 300

#speed of arm in mm/min
INIT_ARM_SPEED = 15000 # default 15000
TRACKING_ARM_SPEED = 1000 # default 1000

# higher value -> larger arm movement in every step. 10 is default. (this affects target distance, not speed)
STEP_SIZE = 10

# ms to wait after init before recording the tracked item reference. Good to give the arm time to move to position.
INIT_WAIT = 5000

# -------------------------------------------------- INIT

#initial the uarm
led_err = LED(1) #red
led_ok = LED(2) # green
led_start = LED(3) # Blue led
led_start.toggle()

if MODE == "arm":
    #initial the uart
    uart = UART(3, 115200)

    #test if the connection is active by a status query
    uart.write("P2500\r\n")
    utime.sleep_ms(100)
    response = uart.readline()
    if response != b'ok\n':
        # wait for the arm to get ready (probably needs to switch active uart, send "M2500" command to it from outside)
        while(response != b'ok\n'):
            print(response)
            response = uart.readline()
            utime.sleep_ms(100)

    print("RX:" + str(response))

    if CAM_POS == "horizontal":
        start_z = 160
    else:
        start_z = 100 # in vertical pos we need space to move up.

    #set the uarm to the default position
    uart.write("G0 X250 Y0 Z")
    uart.write(str(start_z) + " F" + str(INIT_ARM_SPEED) + "\r\n")
    if PRINT_ARM_COMM:
        print("TX:" + "G0 X250 Y0 Z" + str(start_z) + " F" + str(INIT_ARM_SPEED) + "\r\n")

#finish the initialization
led_start.off()

# Reset sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((IMG_WIDTH, IMG_HEIGHT))
sensor.set_pixformat(sensor.GRAYSCALE)

sensor.skip_frames(time = 200)
sensor.set_auto_gain(False, value=100)

def draw_keypoints(img, kpts):
    #print(kpts)
    #img.draw_keypoints(kpts)
    img = sensor.snapshot()
    time.sleep(1000)

time_to_wait = INIT_WAIT

kpts1 = None
# NOTE: uncomment to load a keypoints descriptor from file
#kpts1 = image.load_descriptor("/desc.orb")
#img = sensor.snapshot()
#draw_keypoints(img, kpts1)

# -------------------------------------------------- MAIN LOOP

clock = time.clock()
while (True):
    clock.tick()
    img = sensor.snapshot()
    led_err.off()
    if (kpts1 == None):
        if time_to_wait > 0:
            # wait for the hand to get into position before taking the reference keypoint snapshot.
            utime.sleep_ms(ITER_TIME_MS)
            time_to_wait -= ITER_TIME_MS
            continue
        # NOTE: By default find_keypoints returns multi-scale keypoints extracted from an image pyramid.
        kpts1 = img.find_keypoints(max_keypoints=150, threshold=20, scale_factor=1.35)
        draw_keypoints(img, kpts1)
        led_ok.toggle() # flash green LED when reference snapshot is taken
    else:
        # NOTE: When extracting keypoints to match the first descriptor, we use normalized=True to extract
        # keypoints from the first scale only, which will match one of the scales in the first descriptor.
        kpts2 = img.find_keypoints(max_keypoints=150, threshold=10, normalized=True)

        if (kpts2):

            if MODE == "arm" and PRINT_ARM_COMM:
                response = uart.readline()
                if response:
                    print("RX:" + str(response))

            match = image.match_descriptor(kpts1, kpts2, threshold=85)
            print(kpts2, "matched:%d dt:%d"%(match.count(), match.theta()))

            # only follow if we have at least 5 matches. Otherwise red led turns on.
            if (match.count() >= 5):
                # If we have at least n "good matches"
                # Draw bounding rectangle and cross.
                img.draw_rectangle(match.rect())
                img.draw_cross(match.cx(), match.cy(), size=10)

                coords = list(match.rect()) # y, width, x, height
                #print(coords)

                # calculate arm position adjustment based on distance of tracked item center from image center

                magnitude = STEP_SIZE / 200 # adjustment of the delta. 200 is to scale where we want it, e.g. step 10 = 1/20 magnituted.
                if CAM_POS == "horizontal":
                    delta_x = (coords[3] / 2 + coords[1] - IMG_HEIGHT / 2) * magnitude
                    delta_y = (coords[2] / 2 + coords[0] - IMG_WIDTH / 2) * magnitude
                    delta_z = 0
                else:
                    delta_x = 0
                    delta_y = (IMG_WIDTH / 2 - (coords[2] / 2 + coords[0])) * magnitude
                    delta_z = (IMG_HEIGHT / 2 - (coords[3] / 2 + coords[1])) * magnitude

                if MODE == "arm":
                    #Gcode command, seperated the command because of the limit lenght
                    uart.write("G2204") # code for differential position command
                    uart.write(" X" + str(delta_x))
                    uart.write(" Y" + str(delta_y))
                    uart.write(" Z" + str(delta_z))
                    uart.write(" F" + str(TRACKING_ARM_SPEED) + "\r\n") # speed (mm/min) and newline

                    if PRINT_ARM_COMM:
                        print("TX:" + "G2204" + " X" + str(delta_x) + " Y" + str(delta_y) + " Z" + str(delta_z) + " F" + str(TRACKING_ARM_SPEED) + "\r\n")
                else:
                    print("Moving by X:%f, Y:%f, Z:%f" % (delta_x, delta_y, delta_z))
            else:
                led_err.toggle()
                if MODE == "offline":
                    print("Too few matches, %d" % match.count())
            utime.sleep_ms(ITER_TIME_MS)
            led_ok.off() # in case it was on from first snapshot taking




