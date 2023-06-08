#!/usr/bin/env python3
#coding=utf-8
import time, yaml
from nav_msgs.msg import Path
import tf
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist
from visualization_msgs.msg import MarkerArray, Marker
import math
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import pi

class KalmanFilter():
    def __init__(self,Q=0.00001,R=0.1):
        # Q为这一轮的心里的预估误差
        self.Q = Q
        # R为下一轮的测量误差
        self.R = R
        # Accumulated_Error为上一轮的估计误差，具体呈现为所有误差的累计
        self.Accumulated_Error = 1
        # 初始旧值
        self.kalman_adc_old = -0.4
        self.SCOPE = 50
    def kalman(self,ADC_Value):
        # 新的值相比旧的值差太大时进行跟踪
        if (abs(ADC_Value-self.kalman_adc_old)/self.SCOPE > 0.25):
            Old_Input = ADC_Value*0.382 + self.kalman_adc_old*0.618
        else:
            Old_Input = self.kalman_adc_old
        # 上一轮的 总误差=累计误差^2+预估误差^2
        Old_Error_All = (self.Accumulated_Error**2 + self.Q**2)**(1/2)
        # R为这一轮的预估误差
        # H为利用均方差计算出来的双方的相信度
        H = Old_Error_All**2/(Old_Error_All**2 + self.R**2)
        # 旧值 + 1.00001/(1.00001+0.1) * (新值-旧值)
        kalman_adc = Old_Input + H * (ADC_Value - Old_Input)
        # 计算新的累计误差
        self.Accumulated_Error = ((1 - H)*Old_Error_All**2)**(1/2)
        # 新值变为旧值
        self.kalman_adc_old = kalman_adc
        return kalman_adc


class PurePersuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', log_level=rospy.DEBUG)

        self.KF_X = KalmanFilter(Q=0.0002,R=0.001)
        self.KF_Y = KalmanFilter(Q=0.0002,R=0.001)
        #self.KF_R = KalmanFilter(Q=0.0001,R=0.003)

        self.tf_listener = tf.TransformListener()
        self.currentPath = Path()       #初始化路径
        self.PathOld = Path()

        rospy.loginfo('正在初始化subscriber...')
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback, queue_size = 1)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.pub_marker = rospy.Publisher("/Marker", Marker, queue_size=10)
        self.pub_state = rospy.Publisher("/pure_presuit_state", String, queue_size=10)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) #实例化action服务
        rospy.loginfo('等待move_base action服务器连接...')
        self.move_base.wait_for_server(rospy.Duration(30)) #等待连接服务
        rospy.loginfo('已连接导航服务')  #提示连接成功

        self.currentPath = Path()	#初始化路径

        self.HORIZON = 0.9	#前视距离
        self.L = 0.31    #车辆轴距
        self.Vel_x = 1.5#x方向速度
        self.Vel_x_Old = 0
        self.vx_alpha = 1
        self.Vel_x_min = 0.2
        self.SlowDistance = 0.5	#减速距离
        self.StopDistance = 0.3	#停止距离

        self.R_max = 5 #设置曲率半径限制的最大值，半径大于此值则不限制预瞄距离
        self.R_min_rate = 0.4 #设置曲率半径限制的最小比例，值为0-1

        # 显示相关参数初始化
        self.x_LISH = []
        self.VELA_LISH_FIL = []
        self.R_list = []

        #导航相关参数
        self.states = 1
        self.stop_num = 0
        self.retry_nav_time = 3
        self.CloseDistance = 0.9

        self.PathUpdateTime = 0	#if update path
        self.Max_PathUpdateTime = 0.3
        self.PathOld = Path()

        self.rate = rospy.Rate(30)

		# 等待tf准备完毕
        while not rospy.is_shutdown():
            pose_x, pose_y, pose_yaw = self.get_pos()
            if pose_x != None:
                print("pure_pursuit ready!!!!!")
                break

    # --------------------导航相关函数-----------------------------
    #导航至指定目标点    
    def nav_to(self,x,y,th):#导航函数，添加坐标点,输入x（前）坐标，y（左）坐标，th（平面朝向-360～360度）
        self.states == 2	#重置跟踪状态
        goal = MoveBaseGoal()  #实例化目标点消息
        goal.target_pose.header.frame_id='map'  #设置目标tf
        goal.target_pose.header.stamp = rospy.Time.now()  #获取系统时间戳
        goal.target_pose.pose.position.x=float(x)  #设置x坐标
        goal.target_pose.pose.position.y=float(y)  #设置y坐标
        quaternion=quaternion_from_euler(0.0,0.0,float(th)/180.0*pi) #根据传入欧拉角d转四元数
        goal.target_pose.pose.orientation.x=quaternion[0] #四元数x赋值
        goal.target_pose.pose.orientation.y=quaternion[1] #四元数y赋值
        goal.target_pose.pose.orientation.z=quaternion[2] #四元数z赋值
        goal.target_pose.pose.orientation.w=quaternion[3] #四元数w赋值
        rospy.loginfo('尝试导航去坐标点:('+str(x)+","+str(y)+','+str(th)+')')  #提示连接成功
        self.move_base.send_goal(goal) #发送导航目标

	# 导航到达目标 
    def nav_place_reach(self,x,y,th):
        rospy.logwarn("nav_place_reach start")
        self.nav_to(x,y,th)#发布指定目标点
        rospy.loginfo('navigation goal:('+str(x)+","+str(y)+','+str(th)+')')

        time_start = time.time()	#初始化时间，用于超时后退重新规划
        time_calculate = time.time()
        while not rospy.is_shutdown():
            pose_x, pose_y, pose_yaw = self.get_pos()
            #rospy.loginfo("try calculate ")
            if len(self.currentPath.poses)>1 and pose_x:
                #time_start = time.time()	#初始化时间，用于超时后退重新规划
                self.states = self.calculateTwistCommand(pose_x,pose_y,pose_yaw)
                rospy.loginfo("calculate states is : "+str(self.states))
                if self.states == 1 and (time.time()-time_calculate)>0.8:	#到达目标点
                    rospy.loginfo('reach goal:('+str(x)+","+str(y)+','+str(th)+')')
                    break
            self.PathOld = self.currentPath

            # if (time.time()-self.PathUpdateTime) > self.Max_PathUpdateTime and self.states!=1:
            #     #self.nav_to(x,y,th)#发布指定目标点
            #     #print("time",time.time()-time_start)
            #     twistCmd = Twist()
            #     twistCmd.linear.x = -0.7
            #     twistCmd.angular.z = 0
            #     time_start_back = time.time()
			# 	#后退重新规划
            #     while (not rospy.is_shutdown()) and (time.time()-time_start_back) < 1.25 :
            #         self.cmd_pub.publish(twistCmd)	
            #     #self.nav_to(x,y,th)#发布指定目标点  

            self.rate.sleep()


    def loop(self):
        rate = rospy.Rate(30)
        rospy.logwarn("pure persuit start")
        while not rospy.is_shutdown():
            pose_x, pose_y, pose_yaw = self.get_pos()
            if self.currentPath and pose_x and self.Sroad_state!="start":
                pass
                twistCommand = self.calculateTwistCommand(pose_x,pose_y,pose_yaw)
                self.cmd_pub.publish(twistCommand)
            else:
                self.publish_state("stop")	#发布小车状态
            rate.sleep()

    #读取路径
    def path_callback(self,data):
        self.currentPath = data
        #print("pathold",self.PathOld)
        if self.PathOld.poses != data.poses:
            self.PathUpdateTime = time.time()
            self.PathOld = data
            #print("success get path :",len(data.poses))

    #定义更新坐标函数
    def get_pos(self):  
        #print("try get pos")  
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        #rospy_Time(0)指最近时刻存储的数据
        #得到从 '/map' 到 '/base_link' 的变换，在实际使用时，转换得出的坐标是在 '/base_link' 坐标系下的。
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #rospy.loginfo("get /map to /base_link tf Error")
            return None,None,None
        #global x,y
        (r, p, yaw) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
        x = trans[0]
        y = trans[1]
        #print("success get pos x=%f y=%f ang=%f"%(x,y,yaw))
        return x, y, yaw

    # 发布marker
    def publish_marker(self,x,y):
        #print("发布marker显示局部目标点")
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        # marker.action = Marker.ADD

        marker.id = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        self.pub_marker.publish(marker)

    # 发布跟踪状态
    def publish_state(self,data):
        msg = String()
        msg.data = data
        self.pub_state.publish(msg)

    #计算限制前视距离，使用曲率
    def get_curvature(self,x1,y1,x2,y2,x3,y3):
        dis_1 = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        dis_2 = math.sqrt((y3-y2)**2 + (x3-x2)**2)
        dis_3 = math.sqrt((y3-y1)**2 + (x3-x1)**2)
        R = (dis_1*dis_2*dis_3)/math.sqrt((dis_1+dis_2-dis_3)*(dis_1-dis_2+dis_3)*\
        (-dis_1+dis_2+dis_3)*(dis_1+dis_2+dis_3))

        if R>self.R_max:	#限幅
            R =self.R_max
        R = R/self.R_max	#归一化

        if R < self.R_min_rate :	#限幅
            R = self.R_min_rate
        return R

    # 规划速度	
    # 返回值： -1：报错		0：无路径		1：停止		2：运动中
    def calculateTwistCommand(self,pose_x,pose_y,pose_yaw):	
        try:
                
            lad = 0.0 #look ahead distance accumulator
            targetIndex = len(self.currentPath.poses) - 1	#如果路径长度小于lad，取最后一个点

            # 若无全局路径，停止
            if targetIndex < 1:
                twistCmd = Twist()
                twistCmd.linear.x = 0
                twistCmd.angular.z = 0
                self.cmd_pub.publish(twistCmd)	
                self.Vel_x_Old = 0 
                return 0	#无路径

            # 寻找最远跟踪点
            for i in range(len(self.currentPath.poses)):
                if((i+1) < len(self.currentPath.poses)):
                    this_x = self.currentPath.poses[i].pose.position.x
                    this_y = self.currentPath.poses[i].pose.position.y
                    next_x = self.currentPath.poses[i+1].pose.position.x
                    next_y = self.currentPath.poses[i+1].pose.position.y
                    lad = lad + math.hypot(next_x - this_x, next_y - this_y)#欧里几得距离
                    if(lad > self.HORIZON):
                        targetIndex = i+1	#取距离大于lad的下一个点
                        break

            # 计算曲率半径
            #R_horizon = self.get_curvature(self.currentPath.poses[0].pose.position.x,\
            #								self.currentPath.poses[0].pose.position.y,\
            #								self.currentPath.poses[targetIndex//2].pose.position.x,\
            #								self.currentPath.poses[targetIndex//2].pose.position.y,\
            #								self.currentPath.poses[targetIndex].pose.position.x,\
            #								self.currentPath.poses[targetIndex].pose.position.y)
            #self.R_list.append(R_horizon)
            #print("曲率半径限制为:",R_horizon)
            #print("曲率半径限制为:",self.R_list)

            #R_curvature = self.KF_R.kalman(R_curvature)

            #targetIndex = int(targetIndex*R_horizon/self.HORIZON)

            target_x = self.currentPath.poses[targetIndex].pose.position.x
            target_y = self.currentPath.poses[targetIndex].pose.position.y

            target_x = self.KF_X.kalman(target_x)	#对目标进行卡尔曼滤波，减少波动
            target_y = self.KF_Y.kalman(target_y)

            #print("规划前视距离为：",lad)
            # print("规划前视索引为：",targetIndex)
            # print("规划前视点坐标为：x=%f y=%f"%(target_x,target_y))
            self.publish_marker(target_x,target_y)

            #get angle difference
            alpha = math.atan2(target_y - pose_y, target_x - pose_x) - pose_yaw
            #print("alpha = ",alpha)
            l = math.sqrt(math.pow(pose_x - target_x, 2) + math.pow(pose_y - target_y, 2))
            print("l = ",l)
            

            if(l > self.StopDistance):
                self.stop_num = 0
                theta = math.atan(2 * self.L * math.sin(alpha) / l)
                vel_a = math.tan(theta)*self.Vel_x/self.L
                #print("theta = ",theta)
                #print('vel_a = ',vel_a)

                # vel_a = self.KF.kalman(vel_a)

                if(l < self.SlowDistance):
                    vel_x = (l-self.StopDistance)/(self.SlowDistance-self.StopDistance) * self.Vel_x				
                    vel_a = 0.5*(l-self.StopDistance)/(self.SlowDistance-self.StopDistance) * vel_a
                    if vel_x < self.Vel_x_min:
                        vel_x = self.Vel_x_min
                else:
                    vel_x = self.Vel_x
                vel_x = self.vx_alpha*vel_x + (1-self.vx_alpha)*self.Vel_x_Old
                self.Vel_x_Old = vel_x
                print('vel_x = ',vel_x)
                # #get twist command
                twistCmd = Twist()
                twistCmd.linear.x = vel_x
                twistCmd.angular.z = vel_a

                self.publish_state("moving")	#发布小车状态
                self.cmd_pub.publish(twistCmd)
                return 2	#小车运动中

            else:
                twistCmd = Twist()
                twistCmd.linear.x = 0
                twistCmd.angular.z = 0
                self.stop_num += 1
                if self.stop_num > 3:
                    return 1
                self.publish_state("stop")	#发布小车状态
                self.cmd_pub.publish(twistCmd)
                self.Vel_x_Old = 0 
                return 2	#到达目标停止


            #self.x_LISH.append(target_x)
            # print(self.x_LISH)

        except Exception as e:
            print("except:",e)  
            print("规划失败！！小车停止")
            twistCmd = Twist()
            twistCmd.linear.x = 0
            twistCmd.angular.z = 0

            self.publish_state("stop")	#发布小车状态
            self.Vel_x_Old = 0 
        self.cmd_pub.publish(twistCmd)
        return -1	#规划失败，退出


#读取yaml文件
def yaml_read(yaml_path):
	global goal_dict
	global nav_goals
	f = open(yaml_path)
	cfg =  f.read()
	goal_dict = yaml.load(cfg)
	nav_goals= goal_dict.keys()
	print("goal_dict = ", goal_dict)   #目标列表
	print("nav_goals", nav_goals)  #目标名称列表


#初始化变量
goal_dict = []
nav_goals = []
yaml_path = "/home/wgq/catkin_ws/src/bringup/config/goal.yaml"

#定义发布目标函数，输入为地点名字符
def nav_reach(place):
	place_name=place
	print("尝试导航去:"+place_name)
	goal=place
	if goal in nav_goals:
		print("尝试导航去:"+place_name)
		goal_data=goal_dict[goal]
		print("goal_data[0] : ",goal_data[0])
		qingzhou.nav_place_reach(goal_data[0],goal_data[1],goal_data[2])

if __name__ == '__main__':
    try:
        qingzhou = PurePersuit()
        #qingzhou.Vel_x = 0.8
        qingzhou.CloseDistance = 0.9
        yaml_read(yaml_path)
        cnt = 0

        while not rospy.is_shutdown():
            nav_reach("goal1")

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start pure pursuit node.')
