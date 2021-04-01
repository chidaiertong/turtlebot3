import rospy
from test_pkg.msg import Locatpose
import cv2 as cv
import numpy as np
import math
import time
from numpy import linalg
import os
import rospy

cap = cv.VideoCapture(2)#2
time.sleep(2)

#测试程序运行速度
times_1 = 0
times_2 = 0
times_3 = 0

key_detect = True #默认菱形编队,代号True,False代表直线编队

#定义颜色hsv模型区域
#blue
lower_blue = np.array([100,100,150])  
upper_blue = np.array([108,220,265])
#yellow(purple in fact)
lower_yellow = np.array([112,80,120])
upper_yellow = np.array([130,145,230])
#green
lower_green = np.array([65,80,140])
upper_green = np.array([77,130,230])
#red1
lower_red1 = np.array([0,110,150]) 
upper_red1 = np.array([10,180,260])
#red2
lower_red2 = np.array([170,80,200])
upper_red2 = np.array([180,160,265])

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
mtx=np.array([[1.64308619e+03 ,0.00000000e+00 ,4.32360070e+03],
             [0.00000000e+00 ,1.54161395e+03 ,3.52365674e+02],
             [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])
#旋转
rvec=np.array([[-0.26294849],[-0.28122597],[1.13264505]])  
#平移
tvec=np.array([[1.57610734],[-13.94429451],[110.36889126]])  

#圆心缓存列表
blue_center = []

#中心点初始化
point_blue = [0,0]
point_yellow = [0,0]
point_green = [0,0]
point_red = [0,0] 
#检测到姿态的标志位
circle_flag = False
four_angle = [0.0,0.0,0.0,0.0]
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

def talker(red_xx,red_yy,blue_xx,blue_yy,yellow_xx,yellow_yy,green_xx,green_yy,four_angle,key):

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

    master_msg.red_angle    = four_angle[0]
    master_msg.blue_angle   = four_angle[1]
    master_msg.yellow_angle = four_angle[2]
    master_msg.green_angle  = four_angle[3]
    master_msg.evolution    = key

    #利用发布器发布话题
    location_info_pub.publish(master_msg)
    #打印字符串
    # rospy.loginfo("Publish locat_pose message[%f, %f, %f,%f, %f,%f]", 
    #             master_msg.red_x, master_msg.red_y ,
    #             master_msg.red_yellow_x,master_msg.red_yellow_y ,
    #             master_msg.red_green_x,master_msg.red_green_y)

#-------该函数为测试用，与tb1上的函数相同---------#
def force_calculate(a,b,c,d,xita):#计算距离目标点的方向和大小
    #(a,b,xita)is leader's position and towards
    #(c,d)is follower's position
    xita = xita*3.14/180 #transform unit to rad
    l = 80 #expected distance, Unit : pixel
    x = a - l*math.cos(0.523-xita) - c
    y = b + l*math.sin(0.523-xita) - d 
    xx = x+c
    yy = y+d
    force_angle = 57.29578*math.atan2(y,x)
    derta_x = (xx-c)*(xx-c)
    derta_y = (yy-d)*(yy-d)
    magnitude = math.sqrt(derta_x+derta_y)

    return xx,yy,force_angle,magnitude
#---------------------------------------------#
def lvbo(list1):
    #找到最大值与最小值：
    max1 = np.max(list1)
    min1 = np.min(list1)
    sum1 = np.sum(list1)
    sum1 = sum1 - max1 - min1
    #求平均值
    average = sum1 / 5.0

    return average

def lvbo2(list1):
    #找到最大值与最小值：
    max1 = np.max(list1)
    min1 = np.min(list1)
    sum1 = np.sum(list1)
    sum1 = sum1 - max1 - min1
    #求平均值
    average = int(sum1 / 5.0)

    return average
# def distanceone(pc,mtx,rvecs,tvecs):

#     oc=np.array(np.zeros((3,1)))
#     mtx_n=np.linalg.inv(mtx)   #内参矩阵求逆
#     pc_r=np.dot(mtx_n,pc)   #内参逆矩阵与像素矩阵乘法
#     oc_r=oc
#     st , p = cv.Rodrigues(rvecs)    #旋转矩阵由1*3变3*3
#     rt=np.linalg.inv(st)      #旋转矩阵求逆
#     pc_w=np.dot(rt,(pc_r-tvecs))    
#     oc_w=np.dot(rt,(oc_r-tvecs))
#     dc_w = pc_w - oc_w  #求与平面z=0交点
#     pw=(oc_w/oc_w[2]-dc_w/dc_w[2])*oc_w[2] #约掉z
    
#     return pw

# def distancetwo(point1,point2): #返回两个点的x y距离

#     p1w = distanceone(point1,mtx,rvec,tvec)
#     p2w = distanceone(point2,mtx,rvec,tvec)
#     # d = math.sqrt((p1w[0]-p2w[0])**2+(p1w[1]-p2w[1])**2)
#     dx = p2w[0]-p1w[0]
#     dy = p2w[1]-p1w[1]

#     return dx,dy

def pixel_distance(point1x,point1y,point2x,point2y): #返回两个点的像素距离(无关顺序)

    derta_x = (point1x - point2x)*(point1x - point2x)
    derta_y = (point1y - point2y)*(point1y - point2y)
    pixel_distance1 = math.sqrt(derta_x+derta_y)

    return pixel_distance1

def angle_cal(x,y): #计算向量(x,y)与水平方向的夹角

    angle = 57.29578*math.atan2(y,x)#返回rad,1rad=57.29578
    if angle < 0.0:
        angle = angle + 360.0

    return angle

def video_process(res_color):#输入按位与掩码后的图像,返回hough变换结果

    #转化为灰度图
    gray_color = cv.cvtColor(res_color,cv.COLOR_BGR2GRAY)
    #转化为二值图
    binary_color = cv.threshold(gray_color,0,255,cv.THRESH_BINARY)[1]
    #阈值分割
    canny_color = cv.Canny(binary_color,60,100)
    #hough变换,返回圆的坐标和半径
    circles_result = cv.HoughCircles(canny_color, cv.HOUGH_GRADIENT, 1, 10, np.array([]), 60, 15, 5, 30)

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
                cv.circle(cimg, (hough_result[0][i][0], hough_result[0][i][1]), math.ceil(hough_result[0][i][2]), (0, 0, 255), 3, cv.LINE_AA) #math.ceil是取整，画圆只能用整数
                cv.circle(cimg, (hough_result[0][i][0], hough_result[0][i][1]), 2, (0, 255, 0), 3, cv.LINE_AA)  # draw center of circle
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
    cimg = frame.copy() #画圆的时候用
    #高斯滤波
    frame = cv.GaussianBlur(frame,(5,5),0)
    # Convert BGR to HSV 将BGR转换为HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # 创造四种颜色的掩膜
    mask_blue   = cv.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv.inRange(hsv, lower_yellow, upper_yellow)
    mask_green  = cv.inRange(hsv, lower_green, upper_green)
    mask_red1   = cv.inRange(hsv, lower_red1, upper_red1)
    mask_red2   = cv.inRange(hsv, lower_red2, upper_red2)
    mask_red    = mask_red1 + mask_red2
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

    four_angle[0] = lvbo(red_angle_buffer)
    four_angle[1] = lvbo(blue_angle_buffer)
    four_angle[2] = lvbo(yellow_angle_buffer)
    four_angle[3] = lvbo(green_angle_buffer)

    #测试用,用来计算虚拟力,应该在tb上运行
    xx,yy,force_angle,magnitude = force_calculate(red_x_pub,red_y_pub,blue_x_pub,blue_y_pub,four_angle[0],)
    #计算相对位置信息
    # red_blue = distancetwo(p_red,p_blue) #蓝色相对红色的位置信息
    # red_yellow = distancetwo(p_red,p_yellow)
    # red_green = distancetwo(p_red,p_green)

    print('red:[%d,%d],red_angle:%.1f°'       %(red_x_pub,red_y_pub,four_angle[0]))
    print('blue:[%d,%d],blue_angle:%.1f°'     %(blue_x_pub,blue_y_pub,four_angle[1]))
    print('yellow:[%d,%d],yellow_angle:%.1f°' %(yellow_x_pub,yellow_y_pub,four_angle[2]))
    print('green:[%d,%d],green_angle:%.1f°'   %(green_x_pub,green_y_pub,four_angle[3]))
    xx = math.ceil(xx)
    yy = math.ceil(yy)
    cv.circle(cimg, (xx, yy), 2, (0, 0, 255), 3, cv.LINE_AA)  # draw center of circle
    cv.imshow("detected circles", cimg)
    print('x:%d,y:%d,force_angle:%.1f,magnitude:%.1f'%(xx,yy,force_angle,magnitude))
    try:
        talker(red_x_pub,red_y_pub,blue_x_pub,blue_y_pub,yellow_x_pub,yellow_y_pub,green_x_pub,green_y_pub,four_angle,key_detect)
    except rospy.ROSInterruptException:
        pass
    #<检测程序运行速度的程序>
    times_1 = times_1 + 1
    if times_1 >=100:
        times_2 = times_2 + 1
        times_1 = 0
    if times_2 >=100:
        times_3 = times_3 + 1
        times_2 = 0
    print('times_1:%d,times_2:%d,times_3:%d' %(times_1,times_2,times_3))
    #</检测程序运行速度的程序>

    k = cv.waitKey(5) & 0xFF 
    if k == 107: #K键(键值75->107与标准键值不同)变换 菱形√ 队形
        key_detect = True
    if k == 108: #L键(键值76->108与标准键值不同)变换 直线× 队形
        key_detect = False
    if k == 27: #ESC键(按键值27)退出
        break
    print('key_detect:',key_detect)

cv.destroyAllWindows()