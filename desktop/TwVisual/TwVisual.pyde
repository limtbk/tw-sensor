
add_library('serial')

def setup():
    global sp, buf, ar
    buf = []
    ar = [0]*12
    found = False
    for p in Serial.list():
        print p
    sp = Serial(this, "/dev/tty.SLAB_USBtoUART", 57600)
    size(200, 150)
    frameRate(30)
    background(50)
    colorMode(HSB)
    
def draw():
    global sp, buf
    fill(0x11000000)
    noStroke()
    rectMode(CORNERS)
    rect(0, 0, width, height)
    while sp.available():
        s = sp.read()
        if s>0:
            buf.append(chr(s))
            if s == 13:
                str = "".join(buf)
                buf = []
                nums = str.split(' ')
                if len(nums)>=12:
                    for hnn in range(12):
                        hn = nums[hnn]
                        n = int(hn, 16)
                        ar[hnn] = n
                print ar
                
    for i in range(12):
        x = i%4
        y = i/4
        noStroke()
        fill(ar[i]%256, 255, 255)
        rectMode(CORNERS)
        rect(x*50, y*50, (x+1)*50, (y+1)*50)

    if mousePressed:
        sp.write(ord('c'))
#         sp.write(ord('r'))
        sp.write(ord('n'))
        sp.write(ord('x'))        
