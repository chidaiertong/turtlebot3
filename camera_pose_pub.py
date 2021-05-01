import rospy
from test_pkg.msg import Locatpose
from threading import Timer
import cv2 as cv
import numpy as np
import math
import time
# from numpy import linalg
import os
import rospy
#
#需要矫正本摄像头，因为tf的单位是米，并非像素（不矫正就是粗略的值）
#
cap = cv.VideoCapture(0)#2
time.sleep(2)
#测试程序运行速度
times_1 = 0
times_2 = 0
times_3 = 0

pixel_to_meter = 0.00483

key_detect = True #默认菱形编队,代号True,False代表直线编队
able_to_publish = True
want_to_publish = False
#定义颜色hsv模型区域
#blue----------#
def force_calculate_line_tb2(a,b,c,d,xita):#tb2跟随tb1,即yellow跟随blue
    #(a,b,xita)is leader's position and towards
    #(c,d)is follower's position
    xita = xita*3.14/180 #transform unit to rad
    l = 80 #expected distance, Unit : pixel
    # xx,yy为tb3的目标位置
    xx = a - l*math.cos(xita)
    yy = b - l*math.sin(xita)
    #x,y是tb2(c,d)指向目标位置(xx,yy)的向量
    x = xx - c
    y = yy - d
    force_angle = 57.29578*math.atan2(y,x)
    magnitude = math.sqrt(x*x+y*y)
    return xx,yy,force_angle,magnitude

#-----------tb3----------------
lower_blue = np.array([108,80,140])  #地下一层光照
upper_blue = np.array([116,220,265])
# lower_blue = np.array([105,180,100])  #一楼走廊光照
# upper_blue = np.array([112,220,160]) #1号机

#yellow(purple in fact)
lower_yellow = np.array([125,40,120]) #2号机
upper_yellow = np.array([136,200,250])
#green
# lower_green = np.array([60,30,150])
# upper_green = np.array([77,120,230])

lower_green = np.array([22,30,200])#地下一层光照
upper_green = np.array([36,140,256])

# lower_green = np.array([10,140,200])#一楼走廊光照
# upper_green = np.array([17,180,256]) #3号机

#red1
# lower_red1 = np.array([0,70,150]) 
# upper_red1 = np.array([10,180,260])
#red2
lower_red2 = np.array([166,80,150])# 地下一层光照
upper_red2 = np.array([180,160,256])

# lower_red2 = np.array([166,160,150])#一楼走廊光照
# upper_red2 = np.array([180,200,256])#0号机

#初始化，默认是0
# p_red = np.array([[0],[0],[1.]])
# p_yellow = np.array([[0],[0],[1.]])
# p_blue = np.array([[0],[0],[1.]])
# p_green = np.array([[0],[0],[1.]])

red_xx    = 0
red_yy    = 0
blue_xx   = 0
blue_yy   = 0
yellow_xx = 0
yellow_yy = 0
green_xx  = 0
green_yy  = 0


blue_angle = 0.0
yellow_angle = 0.0
green_angle = 0.0
red_angle =0.0

#内参矩阵
# mtx=np.array([[1.64308619e+03 ,0.00000000e+00 ,4.32360070e+03],
#              [0.00000000e+00 ,1.54161395e+03 ,3.52365674e+02],
#              [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])
# #旋转
# rvec=np.array([[-0.26294849],[-0.28122597],[1.13264505]])  
# #平移
# tvec=np.array([[1.57610734],[-13.94429451],[110.36889126]])  

#圆心缓存列表
blue_center = []

#中心点初始化
point_blue = [0,0]
point_yellow = [0,0]
point_green = [0,0]
point_red = [0,0] 
#检测到姿态的标志位
circle_flag = False
four_angle = [0.00,0.00,0.00,0.00]
four_angle2 = [0.00,0.00,0.00,0.00]
#位姿缓冲数组,大小为7,迭代一周需要约0.22s
red_x_buffer = [0,0,0,0,0,0,0]
red_y_buffer = [0,0,0,0,0,0,0]
blue_x_buffer = [0,0,0,0,0,0,0]
blue_y_buffer = [0,0,0,0,0,0,0]
yellow_x_buffer = [0,0,0,0,0,0,0]
yellow_y_buffer = [0,0,0,0,0,0,0]
green_x_buffer = [0,0,0,0,0,0,0]
green_y_buffer = [0,0,0,0,0,0,0]

red_angle_buffer = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
blue_angle_buffer = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
yellow_angle_buffer = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
green_angle_buffer = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]


# def image_processing(images):

#     frame = cv.GaussianBlur(images,(5,5),0) #滤波
#     hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) #色域转换

#     return hsv
#-------------------自动队形变换----------------------#
# def transform_change(): 
#     #每隔30s改变一次队形
#     global key_detect
#     key_detect = 1 - key_detect
#     Timer(30.0, transform_change) .start()

def talker(red_xx,red_yy,blue_xx,blue_yy,yellow_xx,yellow_yy,green_xx,green_yy,four_angles,key):

    #初始化ROS节点，命名节点为PC_master
    rospy.init_node('PC_master',anonymous=True)
    #创建发布器，向/chatter话题发布String类型消息
    location_info_pub = rospy.Publisher('/locat_pose_pub',Locatpose,queue_size=10) #名称随便
    #创建一个rospy.Rate对象，控制循环频率为1hz
    # rate = rospy.Rate(1)
    # 初始化learning_topic::Person类型的消息
    master_msg = Locatpose()
    master_msg.red_x    = red_xx
    master_msg.red_y    = red_yy
    master_msg.blue_x   = blue_xx
    master_msg.blue_y   = blue_yy
    master_msg.yellow_x = yellow_xx
    master_msg.yellow_y = yellow_yy
    master_msg.green_x  = green_xx
    master_msg.green_y  = green_yy

    master_msg.red_angle    = four_angles[0]
    master_msg.blue_angle   = four_angles[1]
    master_msg.yellow_angle = four_angles[2]
    master_msg.green_angle  = four_angles[3]
    master_msg.evolution    = key

    #利用发布器发布话题
    location_info_pub.publish(master_msg)
    #打印字符串
    # rospy.loginfo("Publish locat_pose message[%f, %f, %f,%f, %f,%f]", 
    #             master_msg.red_x, master_msg.red_y ,
    #             master_msg.red_yellow_x,master_msg.red_yellow_y ,
    #             master_msg.red_green_x,master_msg.red_green_y)

#-------该函数为测试用，与tb1上的函数几乎相同，区别是tb上不返回xx,yy---------#
def force_calculate_tb1(a,b,c,d,xita):#计算距离目标点的方向和大小
    #(a,b,xita)is leader's position and towards
    #(c,d)is follower's position
    xita = xita*3.14/180 #transform unit to rad
    l = 80 #expected distance, Unit : pixel
    xx = a - l*math.sin(xita+1.047) 
    yy = b + l*math.sin(xita+2.618)  
    x = xx - c
    y = yy - d
    force_angle = 57.29578*math.atan2(y,x)
    magnitude = math.sqrt(x*x+y*y)

    return xx,yy,force_angle,magnitude
#----------------tb1的直线编队---------------#
def force_calculate_line_tb1(a,b,c,d,xita):
    #(a,b,xita)is leader's position and towards
    #(c,d)is follower's position
    xita = xita*3.14/180 #transform unit to rad
    l = 80 #expected distance, Unit : pixel
    # xx,yy为tb3的目标位置
    xx = a - l*math.cos(xita)
    yy = b - l*math.sin(xita)
    #x,y是tb3(c,d)指向目标位置(xx,yy)的向量
    x = xx - c
    y = yy - d
    force_angle = 57.29578*math.atan2(y,x)
    magnitude = math.sqrt(x*x+y*y)
    return xx,yy,force_angle,magnitude

#----------------tb2---------------------#
def force_calculate_tb2(a,b,c,d,xita):#菱形，跟随
    #(a,b,xita)is leader's position and towards
    #(c,d)is follower's position
    xita = xita*3.14/180 #transform unit to rad
    l = 80 #expected distance, Unit : pixel
    # xx,yy为tb2的目标位置
    xx = a - l*math.sin(xita+2.094) #tb1 is (0.523-xita)
    yy = b + l*math.sin(xita+3.665) #tb1 is (0.523-xita)
    #x,y是tb2(c,d)指向目标位置(xx,yy)的向量
    x = xx - c
    y = yy - d
    force_angle = 57.29578*math.atan2(y,x)
    magnitude = math.sqrt(x*x+y*y)
    return xx,yy,force_angle,magnitude
#-----------------tb2的直线编队------------#
def force_calculate_line_tb2(a,b,c,d,xita):#tb2跟随tb1,即yellow跟随blue
    #(a,b,xita)is leader's position and towards
    #(c,d)is follower's position
    xita = xita*3.14/180 #transform unit to rad
    l = 80 #expected distance, Unit : pixel
    # xx,yy为tb3的目标位置
    xx = a - l*math.cos(xita)
    yy = b - l*math.sin(xita)
    #x,y是tb2(c,d)指向目标位置(xx,yy)的向量
    x = xx - c
    y = yy - d
    force_angle = 57.29578*math.atan2(y,x)
    magnitude = math.sqrt(x*x+y*y)
    return xx,yy,force_angle,magnitude

#-----------tb3-----------------#
# def force_calculate_tb3(a,b,c,d,e,f):#菱形编队，跟随两个机器人
    #(a,b),(e,f)are two leader's position
    #(a,b)--blue
    #(e,f)--yellow
    #(c,d)--green is follower's position
    l = 80 #expected distance, Unit : pixel
    #判断有无交点
    if (a-e)*(a-e)+(b-f)*(b-f) <= 4*l*l:
        mid_2 = a*a - 2*a*e + b*b - 2*b*f + e*e + f*f
        mid_1 = mid_2 - 4*l*l
        if mid_2 == 0 :
            mid_2 = 1
        mid_3 = b*math.sqrt(-mid_1/mid_2)
        mid_4 = a*math.sqrt(-mid_1/mid_2)
        mid_5 = f*math.sqrt(-mid_1/mid_2)
        mid_6 = e*math.sqrt(-mid_1/mid_2)
        #有交点,求两个交点,计算以(a,b),(e,f)为圆心,l为半径的圆的交点
        x1 = a/2 + e/2 + mid_3/2 - mid_5/2
        x2 = a/2 + e/2 - mid_3/2 + mid_5/2
        y1 = b/2 + f/2 - mid_4/2 + mid_6/2
        y2 = b/2 + f/2 + mid_4/2 - mid_6/2       
        #输出距离tb3距离较近的点
        # xx,yy为tb3的目标位置 
        if (c-x1)*(c-x1)+(d-y1)*(d-y1) <= (c-x2)*(c-x2)+(d-y2)*(d-y2):
            xx = x1
            yy = y1
        else:
            xx = x2
            yy = y2

    else:
        #无交点,输出tb1和tb2的中点
        # xx,yy为tb3的目标位置 
        xx = (a+e)/2
        yy = (b+f)/2
    #x,y是tb3(c,d)指向目标位置(xx,yy)的向量
    x = xx - c
    y = yy - d
    force_angle = 57.29578*math.atan2(y,x)
    magnitude = math.sqrt(x*x+y*y)
    return xx,yy,force_angle,magnitude
# #------------------tb3的直线编队---------------------#
# def force_calculate_line_tb3(a,b,c,d,xita):
#     #(a,b,xita)is leader's position and towards
#     #(c,d)is follower's position
#     xita = xita*3.14/180 #transform unit to rad
#     l = 80 #expected distance, Unit : pixel
#     # xx,yy为tb3的目标位置
#     xx = a - l*math.cos(xita)
#     yy = b - l*math.sin(xita)
#     #x,y是tb3(c,d)指向目标位置(xx,yy)的向量
#     x = xx - c
#     y = yy - d
#     force_angle = 57.29578*math.atan2(y,x)
#     magnitude = math.sqrt(x*x+y*y)
#     return xx,yy,force_angle,magnitude

#----------滤波函数-------------#
def lvbo(list1):#先找中值，然后中值取平均
    #找到最大值与最小值：
    # max1 = np.max(list1)
    # min1 = np.min(list1)
    # sum1 = np.sum(list1)
    # sum1 = sum1 - max1 - min1
    # #求平均值
    # average = sum1 / 5.0

    #求中值
    mid = np.median(list1)

    return mid

def lvbo2(list1):
    #找到最大值与最小值：
    max1 = np.max(list1)
    min1 = np.min(list1)
    sum1 = np.sum(list1)
    sum1 = sum1 - max1 - min1
    #求平均值
    average = int(sum1 / 5.0)

    return average

def pixel_distance(point1x,point1y,point2x,point2y): #返回两个点的像素距离(无关顺序)

    derta_x = (point1x - point2x)*(point1x - point2x)
    derta_y = (point1y - point2y)*(point1y - point2y)
    pixel_distance1 = math.sqrt(derta_x+derta_y)

    return pixel_distance1

def angle_cal(x,y): #计算向量(x,y)与水平方向的夹角 [-180,+180]
    #
    # angle = 57.29578*math.atan2(y,x)#返回°,1rad=57.29578°
    angle = math.atan2(y,x)#返回rad
    # if angle < 0.0:#原为[-180,+180],修改为[0,360],但是为了配合msg需要变回[-180,+180]
    #     angle = angle + 360.0

    return angle

def point_trans(x_blue,y_blue,theta_blue,x_red,y_red,theta_red):#把(x_blue,y_blue,theta_blue)平移后旋转到red点，变成以red为参考点的坐标

    x = x_blue - x_red
    y = y_blue - y_red

    xx = x * math.cos(theta_red) + y * math.sin(theta_red)
    xx = xx * 0.00483#换算成m
    yy = x * math.sin(theta_red) - y * math.cos(theta_red)
    yy = yy * 0.00483
    theta = theta_red - theta_blue
    
    theta = theta * 57.29578#变成deg判断是否在[-180,+180]
    if theta < -180.0:
        theta = theta + 360.0
    if theta > 180.0:
        theta = theta - 360.0
    theta = theta / 57.29578
    #应该换成rad
    return xx,yy,theta

def video_process(res_color):#输入按位与掩码后的图像,返回hough变换结果

    #转化为灰度图
    gray_color = cv.cvtColor(res_color,cv.COLOR_BGR2GRAY)
    #转化为二值图
    binary_color = cv.threshold(gray_color,0,255,cv.THRESH_BINARY)[1]
    #阈值分割
    canny_color = cv.Canny(binary_color,60,100)
    #hough变换,返回圆的坐标和半径
    circles_result = cv.HoughCircles(canny_color, cv.HOUGH_GRADIENT, 1, 10, np.array([]), 60, 15, 5, 20)

    return circles_result

def video_process_color(res_color):#输入按位与掩码后的图像,返回hough变换结果

    #转化为灰度图
    gray_color = cv.cvtColor(res_color,cv.COLOR_BGR2GRAY)
    cv.imshow('res_color',res_color)
    #转化为二值图
    binary_color = cv.threshold(gray_color,0,255,cv.THRESH_BINARY)[1]
    cv.imshow('binary_color',binary_color)
    #阈值分割
    canny_color = cv.Canny(binary_color,60,100)
    cv.imshow('canny',canny_color)
    #hough变换,返回圆的坐标和半径
    circles_result = cv.HoughCircles(canny_color, cv.HOUGH_GRADIENT, 1, 10, np.array([]), 60, 15, 5, 20)

    return circles_result

def locate_and_pose(hough_result):
    circle_flag = False
    point_x = 0
    point_y = 0
    color_angle = 0.0
    if hough_result.ndim == 3: #矩阵维度是3，空矩阵返回2
        circles_number = hough_result.shape[1] #第二个值,是圆的个数
        # blue_center = []
        if circles_number>1:
            for i in range(circles_number):
                #画所有的圆
                cv.circle(cimg, (hough_result[0][i][0], hough_result[0][i][1]), math.ceil(hough_result[0][i][2]), (0, 0, 255), 2, cv.LINE_AA) #math.ceil是取整，画圆只能用整数
                cv.circle(cimg, (hough_result[0][i][0], hough_result[0][i][1]), 1, (0, 255, 0), 2, cv.LINE_AA)  # draw center of circle
                #cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
            #依次计算第j个圆与n-j个圆的圆心像素距离
            for j in range(circles_number-1): #j从0开始,到len(blue_center)-1停止
                for jj in range(j+1,circles_number):
                    pixel_juli = pixel_distance(hough_result[0][j][0],hough_result[0][j][1],hough_result[0][jj][0],hough_result[0][jj][1])
                    if pixel_juli>15 and pixel_juli<30: #像素距离合适,则输出姿态值
                        circle_flag = True
                        #判断圆半径大小
                        if hough_result[0][j][2]>hough_result[0][jj][2]:
                            #则j_blue是大圆
                            point_big = hough_result[0][j]
                            point_small = hough_result[0][jj]
                        else:
                            #则jj_blue是大圆
                            point_big = hough_result[0][jj]
                            point_small = hough_result[0][j]
                        
                        point_x = math.ceil(point_big[0])
                        point_y = math.ceil(point_big[1])
                        #返回值,中心坐标
                        #car_center = np.array([[point_x],[point_y],[1.0]]) 

                        # print("circles_big:",point_big[2])#大圆半径(调试半径参数用)
                        # print("circles_small:",point_small[2])#小圆半径
                        
                        deerta_x = point_big[0] - point_small[0]
                        deerta_y = point_big[1] - point_small[1]
                        #返回值,角度
                        color_angle = angle_cal(deerta_x,deerta_y)
                        # print("angle",color_angle)

    return point_x,point_y,color_angle,circle_flag



while(1):
    # Take each frame 取每一帧
    frame = cap.read()[1]
    # frame = cv.imread('233.jpg',cv.IMREAD_COLOR)

    cimg = frame.copy() #画圆的时候用
    #高斯滤波
    # frame = cv.GaussianBlur(frame,(5,5),0)
    # Convert BGR to HSV 将BGR转换为HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    cv.imshow("hsv", hsv)

    # 创造四种颜色的掩膜
    mask_blue   = cv.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv.inRange(hsv, lower_yellow, upper_yellow)
    mask_green  = cv.inRange(hsv, lower_green, upper_green)
    # mask_red1   = cv.inRange(hsv, lower_red1, upper_red1)
    mask_red2   = cv.inRange(hsv, lower_red2, upper_red2)
    # mask_red    = mask_red1 + mask_red2
    mask_red    = mask_red2
    # 按位与掩码和原始图像
    res_blue   = cv.bitwise_and(frame,frame, mask= mask_blue)
    res_red    = cv.bitwise_and(frame,frame, mask= mask_red)
    res_yellow = cv.bitwise_and(frame,frame, mask= mask_yellow)
    res_green  = cv.bitwise_and(frame,frame, mask= mask_green)
    #返回hough变换结果
    circles_blue   = video_process(res_blue)
    circles_yellow = video_process(res_yellow)
    circles_green  = video_process(res_green)
    circles_red    = video_process(res_red)
    #判断并计算坐标中心和姿态

    temporary_red_xx,temporary_red_yy,temporary_red_angle,red_flag = locate_and_pose(circles_red)
    if red_flag == True:
        red_xx    = temporary_red_xx
        red_yy    = temporary_red_yy
        red_angle = temporary_red_angle
        del red_x_buffer[0]
        del red_y_buffer[0]
        del red_angle_buffer[0]
        red_x_buffer.append(red_xx)
        red_y_buffer.append(red_yy)
        red_angle_buffer.append(red_angle)


    temporary_blue_xx,temporary_blue_yy,temporary_blue_angle,blue_flag = locate_and_pose(circles_blue)
    if blue_flag == True:
        blue_xx    = temporary_blue_xx
        blue_yy    = temporary_blue_yy
        blue_angle = temporary_blue_angle
        #更新buffer,删除第1个数据,在buffer后添加新的数据
        del blue_x_buffer[0]
        del blue_y_buffer[0]
        del blue_angle_buffer[0]
        blue_x_buffer.append(blue_xx)
        blue_y_buffer.append(blue_yy)
        blue_angle_buffer.append(blue_angle)

    temporary_yellow_xx,temporary_yellow_yy,temporary_yellow_angle,yellow_flag = locate_and_pose(circles_yellow)
    if yellow_flag == True:
        yellow_xx    = temporary_yellow_xx
        yellow_yy    = temporary_yellow_yy
        yellow_angle = temporary_yellow_angle
        del yellow_x_buffer[0]
        del yellow_y_buffer[0]
        del yellow_angle_buffer[0]
        yellow_x_buffer.append(yellow_xx)
        yellow_y_buffer.append(yellow_yy)
        yellow_angle_buffer.append(yellow_angle)
    
    temporary_green_xx,temporary_green_yy,temporary_green_angle,green_flag = locate_and_pose(circles_green)
    if green_flag == True:
        green_xx    = temporary_green_xx
        green_yy    = temporary_green_yy
        green_angle = temporary_green_angle
        del green_x_buffer[0]
        del green_y_buffer[0]
        del green_angle_buffer[0]
        green_x_buffer.append(green_xx)
        green_y_buffer.append(green_yy)
        green_angle_buffer.append(green_angle)
    

    #只测试了red blue的效果,可以的话继续增加yellow和green
    red_x_pub    = lvbo2(red_x_buffer) 
    red_y_pub    = lvbo2(red_y_buffer) 
    blue_x_pub   = lvbo2(blue_x_buffer) 
    blue_y_pub   = lvbo2(blue_y_buffer) 
    yellow_x_pub = lvbo2(yellow_x_buffer) 
    yellow_y_pub = lvbo2(yellow_y_buffer) 
    green_x_pub  = lvbo2(green_x_buffer) 
    green_y_pub  = lvbo2(green_y_buffer) 


    # red_x_pub    = lvbo2(red_x_buffer) * pixel_to_meter
    # red_y_pub    = lvbo2(red_y_buffer) * pixel_to_meter
    # blue_x_pub   = lvbo2(blue_x_buffer) * pixel_to_meter
    # blue_y_pub   = lvbo2(blue_y_buffer) * pixel_to_meter
    # yellow_x_pub = lvbo2(yellow_x_buffer) * pixel_to_meter
    # yellow_y_pub = lvbo2(yellow_y_buffer) * pixel_to_meter
    # green_x_pub  = lvbo2(green_x_buffer) * pixel_to_meter
    # green_y_pub  = lvbo2(green_y_buffer) * pixel_to_meter

    four_angle[0] = lvbo(red_angle_buffer)
    four_angle[1] = lvbo(blue_angle_buffer)
    four_angle[2] = lvbo(yellow_angle_buffer)
    four_angle[3] = lvbo(green_angle_buffer)

    #-------将绝对坐标转换为以tb0为原点的相对坐标-----#
    #应用转轴公式变换点，角度单独计算!!!!!注意顺序
    blue_x_pub,blue_y_pub,four_angle[1] = point_trans(blue_x_pub,blue_y_pub,four_angle[1],red_x_pub,red_y_pub,four_angle[0])
    yellow_x_pub,yellow_y_pub,four_angle[2] = point_trans(yellow_x_pub,yellow_y_pub,four_angle[2],red_x_pub,red_y_pub,four_angle[0])
    green_x_pub,green_y_pub,four_angle[3] = point_trans(green_x_pub,green_y_pub,four_angle[3],red_x_pub,red_y_pub,four_angle[0])
    
    #---------------------------------#
    #测试用,用来计算虚拟力,应该在tb上运行
    if key_detect == True:#菱形
        xx_tb1,yy_tb1,force_angle_tb1,magnitude_tb1 = force_calculate_tb1(red_x_pub,red_y_pub,blue_x_pub,blue_y_pub,four_angle[0])
        xx_tb2,yy_tb2,force_angle_tb2,magnitude_tb2 = force_calculate_tb2(red_x_pub,red_y_pub,yellow_x_pub,yellow_y_pub,four_angle[0])
        # xx_tb3,yy_tb3,force_angle_tb3,magnitude_tb3 = force_calculate_tb3(blue_x_pub,blue_y_pub,green_x_pub,green_y_pub,yellow_x_pub,yellow_y_pub)
    else:#直线
        xx_tb1,yy_tb1,force_angle_tb1,magnitude_tb1 = force_calculate_line_tb1(red_x_pub,red_y_pub,blue_x_pub,blue_y_pub,four_angle[0])
        xx_tb2,yy_tb2,force_angle_tb2,magnitude_tb2 = force_calculate_line_tb2(blue_x_pub,blue_y_pub,yellow_x_pub,yellow_y_pub,four_angle[1])
        # xx_tb3,yy_tb3,force_angle_tb3,magnitude_tb3 = force_calculate_line_tb3(yellow_x_pub,yellow_y_pub,green_x_pub,green_y_pub,four_angle[2])


    xx_tb1 = math.ceil(xx_tb1)
    xx_tb2 = math.ceil(xx_tb2)
    # xx_tb3 = math.ceil(xx_tb3)
    yy_tb1 = math.ceil(yy_tb1)
    yy_tb2 = math.ceil(yy_tb2)
    # yy_tb3 = math.ceil(yy_tb3)

    cv.circle(cimg, (xx_tb1, yy_tb1), 2, (255, 0, 0), 3, cv.LINE_AA)  # draw center of circle (0 0 255)点(B G R)
    cv.circle(cimg, (xx_tb2, yy_tb2), 2, (128, 0, 128), 3, cv.LINE_AA)  # draw center of circle (0 255 0)
    # cv.circle(cimg, (xx_tb3, yy_tb3), 2, (0, 255, 0), 3, cv.LINE_AA)  # draw center of circle (255 0 0)

    cv.imshow("detected circles", cimg)
    # print('x:%d,y:%d,force_angle:%.1f,magnitude:%.1f'%(xx_tb1,yy_tb1,force_angle,magnitude))
    print('red:[%f,%f],red_angle:%.1f°'       %(red_x_pub,red_y_pub,four_angle[0]))
    print('blue:[%f,%f],blue_angle:%.1f°'     %(blue_x_pub,blue_y_pub,four_angle[1]))
    print('purple:[%f,%f],purple_angle:%.1f°' %(yellow_x_pub,yellow_y_pub,four_angle[2]))
    print('orange:[%f,%f],orange_angle:%.1f°'   %(green_x_pub,green_y_pub,four_angle[3]))
    #<检测程序运行速度的程序>
    # times_1 = times_1 + 1
    # if times_1 >=100:
    #     times_2 = times_2 + 1
    #     times_1 = 0
    # if times_2 >=100:
    #     times_3 = times_3 + 1
    #     times_2 = 0
    # print('times_1:%d,times_2:%d,times_3:%d' %(times_1,times_2,times_3))
    #</检测程序运行速度的程序>


    #-------------------加入判断是否稳定的程序------------------------#

    if want_to_publish == True: #感觉比较稳定了,希望发送位姿信息,以后一直是True，保持发送
        if able_to_publish == True:
            red_x_pub2   = red_x_pub
            red_y_pub2    = red_y_pub
            blue_x_pub2   = blue_x_pub
            blue_y_pub2   = blue_y_pub
            yellow_x_pub2 = yellow_x_pub
            yellow_y_pub2 = yellow_y_pub
            green_x_pub2  = green_x_pub
            green_y_pub2  = green_y_pub

            four_angle2[0] = four_angle[0]
            four_angle2[1] = four_angle[1]
            four_angle2[2] = four_angle[2]
            four_angle2[3] = four_angle[3]

            able_to_publish = False
        try:
            talker(red_x_pub2,red_y_pub2,blue_x_pub2,blue_y_pub2,yellow_x_pub2,yellow_y_pub2,green_x_pub2,green_y_pub2,four_angle2,key_detect)
        except rospy.ROSInterruptException:
            pass 
    #--------------------------------------------------------------#

    k = cv.waitKey(5) & 0xFF 
    print(k)#打印键值
    if k == 108: #Lingxing键(键值75->107与标准键值不同)变换 菱形√ 队形
        key_detect = True
    if k == 114: #Reset键(键值与标准键值不同)重新发送所有信息
        able_to_publish = True
    if k == 122: #Zhixian键(键值与标准键值不同)变换 直线× 队形
        key_detect = False
    if k == 112: #P键(第一次发送位置和队形信息，以后只发送队形信息,位姿信息被锁)
        want_to_publish = True
        # print(k)#打印键值
        if key_detect == True:
            print('菱形编队')
        else:
            print('直线编队')

    if k == 27: #ESC键(按键值27)退出
        break

cv.destroyAllWindows()

#当前高度，172像素大概1m，约合1像素0.0058m