import sensor, image, time, math,pyb, json
from pyb import UART, Timer
from struct import pack
# Initialize camera
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 200)
sensor.set_auto_gain(True) # must be turned off for color tracking
sensor.set_auto_whitebal(True) # must be turned off for color tracking
clock = time.clock()
goal_x=80;
goal_y=60;

#----------------------------------------color threshold set
GRAYSCALE_THRESGOLD = [(0,100)]#((0, 37, -85, 22, -5, 43))
#GRAYSCALE_THRESGOLDl = [(0,100)]  #grayscale是一个函数 功能是使用最大类间方差法找到图片的一个合适的阈值
low_threshold = (0, 100)   #低临界阈值
high_threshold = (150, 200)#高临界阈值
#THRESGOLD = ((0, 37, -85, 22, -5, 43))#black

#---------------------------------------ROI area set #roi区域设置
roi_cap=[
         (0  , 0  , 20 , 120,0),#left  1     #前后左右中五个roi
         (20 , 0  , 120, 30 ,0),#up    2
         (20 , 40 , 120, 40 ,0),#mid   3
         (20 , 90 , 120, 30 ,0),#down  4
         (140, 0  , 20 , 120,0),#right 5
         ]
weight_sum = 0
for r in roi_cap:  weight_sum+= r[4]

#----------------------------------------right angle set  右直角设置
enable_lens_corr = False  # turn on for straighter lines...打开直线线
right_angle_threshold = (70, 90) #右角度阈值
forget_ratio = 0.8
move_threshold = 5   #移动阈值

#---------------------------------------calculate angle //预测角度 计算角度
right_angle_threshold = (70, 90)
binary_threshold = [(0, 10)]   #二元阈值
forget_ratio = 0.8
move_threshold = 5
low_threshold = (0,100)
def calculate_angle(line1, line2):
    # 利用四边形的角公式， 计算出直线夹角
    angle  = (180 - abs(line1.theta() - line2.theta()))
    if angle > 90:
        angle = 180 - angle
    return angle


def is_right_angle(line1, line2):
    global right_angle_threshold
    # 判断两个直线之间的夹角是否为直角
    angle = calculate_angle(line1, line2)

    if angle >= right_angle_threshold[0] and angle <=  right_angle_threshold[1]:
        # 判断在阈值范围内
        return True
    return False

def find_verticle_lines(lines):  #找出垂直的线
    line_num = len(lines)
    for i in range(line_num -1):
        for j in range(i, line_num):
            if is_right_angle(lines[i], lines[j]):
                return (lines[i], lines[j])
    return (None, None)


def calculate_intersection(line1, line2):
    # 计算两条线的交点
    a1 = line1.y2() - line1.y1()
    b1 = line1.x1() - line1.x2()
    c1 = line1.x2()*line1.y1() - line1.x1()*line1.y2()

    a2 = line2.y2() - line2.y1()
    b2 = line2.x1() - line2.x2()
    c2 = line2.x2() * line2.y1() - line2.x1()*line2.y2()

    if (a1 * b2 - a2 * b1) != 0 and (a2 * b1 - a1 * b2) != 0:
        cross_x = int((b1*c2-b2*c1)/(a1*b2-a2*b1))
        cross_y = int((c1*a2-c2*a1)/(a1*b2-a2*b1))
        return (cross_x, cross_y)
    return (-1, -1)

def draw_cross_point(cross_x, cross_y):
    img.draw_cross(cross_x, cross_y)
    img.draw_circle(cross_x, cross_y, 5)
    img.draw_circle(cross_x, cross_y, 10)
# All lines also have `x1()`, `y1()`, `x2()`, and `y2()` methods to get their end-points
# and a `line()` method to get all the above as one 4 value tuple for `draw_line()`.

#-----------------------------------------ROI area point position
left_x=0;
left_y=0;
up_x=0;
up_y=0;
mid_x=0;
mid_y=0;
down_x=0;
down_y=0;
right_x=0;
right_y=0;
i=0;#erport real time area
old_cross_x = 0;
old_cross_y = 0;
cross_x = 0;
cross_y = 0;
#----------------------------------------ROI area check flag
center_flag1=0;
center_flag2=0;
center_flag3=0;
center_flag4=0;
center_flag5=0;

#
#----------------------------------------uart init
uart = UART(3, 115200)                         # i使用给定波特率初始化
uart.init(115200, bits=8, parity=None, stop=1) # 使用给定参数初始化
#uart.read(10)       # read 10 characters, returns a bytes object 读取10字符，返回一个字节对象
#uart.read()         # read all available characters 读取所有可用字符
#uart.readline()     # read a line 读取一条线
#uart.readinto(buf)  # read and store into the given buffer 读取并存入缓冲区
coordinate=0

#------------------------------------------uart send data
angle_offset=0;
move_x=0;
move_y=0;

#------------------------------------------define figure angle  定义圆角
def calculate_angle_free (point1_x,point1_y,point2_x,point2_y):
    angel=0;
    if((point1_y-point2_y)!=0):
      angel=(point1_x-point2_x)/(point2_y-point1_y);
      angel=math.atan(float(angel))
      angel=math.degrees(angel)
    return angel

#--------------------------------------line filter
def line_filter_copy(src, dst):
    for i in range(0, len(dst), 1):
        dst[i] = src[i<<1]

# Segment the image by following thresholds.按照阈值分割图像。
# Note source is YUYV destination is 1BPP Grayscale
def line_filter_bw(src, dst):
    for i in range(0, len(dst), 1):
        if (src[i<<1] > 200 and src[i<<1] < 255):
            dst[i] = 0xFF
        else:
            dst[i] = 0x00

#---------------------------------------core filter
kernel_size = 1 # 3x3==1, 5x5==2, 7x7==3, etc.

kernel = [-2, -1,  0, \
          -1,  1,  1, \
           0,  1,  2]
flag1 = 0
flag3 = 0
flag = 1
cross_flag = 0
led4 = pyb.LED(4)
#------------------------------------------main
while(True):
    clock.tick()
    #img = sensor.snapshot().lens_corr(1.8)
    img = sensor.snapshot() # Take a picture and return the image.
    #for i in range(100):
            #img = sensor.snapshot()
            #img.binary([low_threshold])
       # Test high threshold
    #for i in range(100):
            #img = sensor.snapshot()
            #img.binary([high_threshold])
        # Test not low threshold

    img.erode(2,0)
    img.gaussian(1)
    img.lens_corr(1.5)
        # Test not high threshold
    #for i in range(100):
        #    img = sensor.snapshot()
         #   img.binary([high_threshold], invert = 1)

    circles = img.find_circles(threshold = 2500, x_margin =70, y_margin = 70, r_margin = 70,r_min = 5, r_max = 150, r_step = 1)
    if circles :
          center_x_rect = goal_x - circles[0].x()
          center_y_rect = goal_y - circles[0].y()
          data = bytearray([0x7F,0x7E])
          uart.write(data)

          data = bytearray([0x02, 8])
          uart.write(data)

          data = pack('f', center_y_rect)
          uart.write(data)

          data = pack('f', center_x_rect)
          uart.write(data)

          data = bytearray([center_x_rect, center_y_rect])
          print(center_x_rect, center_y_rect)
          uart.write(data)
          img.draw_circle(circles[0].x(), circles[0].y(), circles[0].r(), color = (255, 0, 0))
    else:
        for r in roi_cap:
            i+=1;
            blobs = img.find_blobs(GRAYSCALE_THRESGOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.

            if blobs:#
                # Find the blob with the most pixels.
                largest_blob = max(blobs, key=lambda b: b.pixels())
                if(i==1):#left rectangle
                  if(largest_blob[3]<=15 and largest_blob[3]>=2):#排除瑕疵点
                    if(largest_blob[2]>=2):
                        left_x=largest_blob.cx();
                        left_y=largest_blob.cy();
                        center_flag1=1;#left flag
                        img.draw_rectangle(largest_blob.rect())
                        img.draw_cross(left_x,left_y)
                elif(i==2):#up rectangle
                    if(largest_blob[2]<=30 and largest_blob[2]>=2):#排除瑕疵点
                       if(largest_blob[3]>=0):
                         up_x=largest_blob.cx();
                         up_y=largest_blob.cy();
                         center_flag2=1;#up flag
                         img.draw_rectangle(largest_blob.rect())
                         img.draw_cross(up_x,up_y)
                elif(i==3):#mid rectangle
                    if(largest_blob[2]<=35 and largest_blob[2]>=2):#排除瑕疵点
                      if(largest_blob[3]>=0):
                        mid_x=largest_blob.cx();
                        mid_y=largest_blob.cy();
                        center_flag3=1;#mid flag
                        img.draw_rectangle(largest_blob.rect())
                        img.draw_cross(mid_x,mid_y)
                elif(i==4):#down rectangle
                    if(largest_blob[2]<=20 and largest_blob[2]>=2):#排除瑕疵点
                      if(largest_blob[3]>=5):
                        down_x=largest_blob.cx();
                        down_y=largest_blob.cy();
                        center_flag4=1;#down flag
                        img.draw_rectangle(largest_blob.rect())
                        img.draw_cross(down_x,down_y)
                elif(i==5):#right rectangle
                    if(largest_blob[3]<=15 and largest_blob[3]>=2):#排除瑕疵点
                      if(largest_blob[2]>=2):
                        right_x=largest_blob.cx();
                        right_y=largest_blob.cy();
                        center_flag5=1;#right flag
                        img.draw_rectangle(largest_blob.rect())
                        img.draw_cross(right_x,right_y)



    #---------------------------------------draw right

        lines =  img.find_lines(threshold = 2000, theta_margin = 40, rho_margin = 20, roi=(5, 5, 150,110))
        # 如果画面中有两条直线
        if len(lines) >=2:
            (line1, line2) = find_verticle_lines(lines)
            if (line1 == None or line2 == None):
                # 没有垂直的直线
                draw_cross_point(old_cross_x, old_cross_y)
                continue
            # 画线
             #img.draw_line(line1.line(), color = (255, 0, 0))
             #img.draw_line(line2.line(), color = (255, 0, 0))

            # 计算交点
            (cross_x, cross_y) = calculate_intersection(line1, line2)
            cross_flag = 1
            #print("cross_x:  %d, cross_y: %d"%(old_cross_x, old_cross_y))

            if cross_x != -1 and cross_y != -1:
                if abs(cross_x - old_cross_x) < move_threshold and abs(cross_y - old_cross_y) < move_threshold:
                    # 小于移动阈值， 不移动
                    pass
                else:
                    old_cross_x = int(old_cross_x * (1 - forget_ratio) + cross_x * forget_ratio)
                    old_cross_y = int(old_cross_y * (1 - forget_ratio) + cross_y * forget_ratio)

            draw_cross_point(old_cross_x, old_cross_y)

    #----------------------------------figure out uart send data
        if lines:
            move_x=goal_x-mid_x;
            move_y=goal_y-mid_y;
            if   center_flag3 and center_flag2 :
               angle_offset=calculate_angle_free(up_x,up_y,mid_x,mid_y)
            elif center_flag3 and center_flag1 :
               angle_offset=calculate_angle_free(left_x,left_y,mid_x,mid_y)
            elif center_flag3 and center_flag5 :
               angle_offset=calculate_angle_free(right_x,right_y,mid_x,mid_y)

            if angle_offset < -50 :
               angle_offset = -90
            elif angle_offset > 50 :
               angle_offset = 90

            data = bytearray([0x7F,0x7E])
            uart.write(data)

            data = bytearray([3, 16])
            uart.write(data)

            data = pack('f', angle_offset)
            uart.write(data)
            data = pack('f', move_x)
            uart.write(data)
            data = pack('f', cross_x)
            uart.write(data)
            data = pack('f', cross_y)
            uart.write(data)
            print('angle_offset: %d, move_x: %d, cross_x: %d, cross_y: %d'%(angle_offset,move_x, cross_x,  cross_y))
            '''
            data = bytearray([flag1, abs(int(angle_offset)),  flag2, abs(move_x), cross_flag, cross_x, cross_y])
            print(angle_offset, move_x, cross_flag, cross_x, cross_y)
            uart.write(data)
            '''

    #---------------------------------clear flag
    center_flag1=0;
    center_flag2=0;
    center_flag3=0;
    center_flag4=0;
    center_flag5=0;
    cross_flag = 0
    i=0;
    j=0;









