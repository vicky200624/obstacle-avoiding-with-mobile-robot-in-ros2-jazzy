import numpy as np
import rclpy
import array
import math
from rclpy.node import Node
#has rclpy functionalities
from sensor_msgs.msg import LaserScan
#has ranges and intensities
from geometry_msgs.msg import Twist
#has linear and angular velocities
from nav_msgs.msg import Odometry
#has position and orientation,velocity
from tf_transformations import euler_from_quaternion
#to convert quaternions to euler angles
#convert it to the rotation angles
import time


#used to send twist control commands
topic1='/cmd_vel'
#used to receive laser pose sensor measurements
topic2='/odom'
#used to receive the lidar scans
topic3='/scan'

class ControllerNode(Node):
    def __init__(self,xdu,ydu,kau,kru,kthetau,gstaru,eps_orientu,eps_controlu):
        super().__init__('controller_node')
        #desired point
        self.xdp=xdu
        self.ydp=ydu
        #control parameters for attraction force
        self.kap=kau
        #control parameters for repulsion force
        self.krp=kru
        #control parameter for orientation controller
        self.kthetap=kthetau
        #limiting influence of the repulsive function
        self.gstarp=gstaru
        #tolarance for performing the first orientation adjustment and control
        self.eps_orient=eps_orientu
        #turning off the controller when reached the goal
        self.eps_control=eps_controlu

        self.OdometryMsg=Odometry()
        self.LidarMsg=LaserScan()

        #intial time
        self.initialTime=time.time()
        #time when odometry message is received
        self.msgOdometryTime=time.time()
        #time when lidar message is received
        self.msgLidarTime=time.time()

        #control message we sending
        self.controlVel=Twist()

        #control value is zero at the beginning

        self.controlVel.linear.x=0.0
        self.controlVel.linear.y=0.0
        self.controlVel.linear.z=0.0

        self.controlVel.angular.x=0.0
        self.controlVel.angular.y=0.0
        self.controlVel.angular.z=0.0

        # control publisher, used to send the control signals
        self.ControlPublisher = self.create_publisher (Twist,
                                                        topic1,
                                                        10) 
        # subscriber
        # SensorCallbackPose is a function used to receive the Odometry sensor messages
        self.PoseSubscriber=self.create_subscription (Odometry,
                                                        topic2,
                                                        self. SensorCallbackPose,
                                                        10)
        # SensorCallbackLidar is a function used to receive the Odometry sensor messages
        self.LidarSubscriber=self.create_subscription (LaserScan,
                                                        topic3,
                                                        self. SensorCallbackLidar,
                                                        10)
        #control frquency 
        self.period=0.05
        
        #controlfunction for sending and calculating the control action
        self.timer = self.create_timer(self.period, self.ControlFunction)

    #orientation and angular velocity
    def orientationError(self,theta_,thetad_):
        # theta in 3st quadrant, thetad in 2th quadrant
        if (thetad_> np. pi/2) and (thetad_<= np.pi):
            if (theta_> np. pi) and (theta_<= np. pi/2):
                theta_=theta_+2*np.pi
        # theta in 2nd quadrant, thetad in 3rd quadrant
        if (theta_> np. pi/2) and (theta_ <= np.pi):
            if (thetad_> np. pi) and (thetad_<= np. pi/2):
                thetad_=thetad_+2*np.pi
        errorOrientation=thetad_-theta_
        return errorOrientation
    
    #for receiving sensor pose
    def SensorCallbackPose(self,receivedMsg):
        self.OdometryMsg=receivedMsg
        self.msgOdometryTime=time.time()
    
    #for receiving lidar scans
    def SensorCallbackLidar(self,receivedMsg):
        self.LidarMsg=receivedMsg
        self.msgLidarTime=time.time()

    #calculating the control action and sending it
    def ControlFunction(self):
        #attractive force
        ka=self.kap
        #repulsive force
        kr=self.krp
        #orientation controller 
        ktheta=self.kthetap
        #limiting influence of the repulsive function
        gstar=self.gstarp
        #extraxt desired position coordinates
        xd=self.xdp
        yd=self.ydp
        #extract current position coordinates,does not give exact so  get quaternions
        x=self.OdometryMsg.pose.pose.position.x
        y=self.OdometryMsg.pose.pose.position.y
        #extract  orientation in quaternions
        quat = self.OdometryMsg.pose.pose.orientation
        #list of quaternions
        quatl= (quat.x, quat.y, quat.z, quat.w)
        
        #convert quaternions to euler angles
        (roll, pitch, yaw)= euler_from_quaternion(quatl)
        #only yaw is hanging
        theta=yaw
        #lidar measurements and parameters
        LidarRanges=np.array(self.LidarMsg.ranges)
        #angle for which the lidar starts measuring
        angle_min=self.LidarMsg.angle_min
        #incremental angle between each measurement
        angle_increment=self.LidarMsg.angle_increment
        
        #attractive force 
        vectorD=np.array([[x - xd], [y - yd]])
        gradUa= ka*vectorD
        AF=-gradUa

        #indices of the lidar measurements that are not infinite
        indices_not_inf= np.where(np.isfinite(LidarRanges))[0]
        #if there is obstacle detected set it as true
        obstacleYES=np.any(np.isinf(indices_not_inf))

        if (obstacleYES):
            #difference between consecutive indices
            diff_array = np.diff(indices_not_inf)
            #find indices where the difference is greater than 1
            split_indices = np.where(diff_array > 1)[0] + 1
            #every sudarray to one osbatacle
            partitioned_arrays = np.split(indices_not_inf, split_indices)
            #angle of all ray
            angles=angle_min+indices_not_inf*angle_increment+theta
            #positions of all points detected by lidar
            distances=LidarRanges[indices_not_inf]
            #all positions in global frame
            x0=x*np.ones(distances.shape)+distances*np.cos(angles)
            y0=y*np.ones(distances.shape)+distances*np.sin(angles) 

            #· compute·minimal distances to obstacles and corresponding angles
            #this list contains the mininal distances to obstacles I
            #· that is, it contains g values
            min_distances=[]
            # this list contains the corresponding angles
            min_distances_angles=[]
            for i in range(len (partitioned_arrays)):
                tmpArray=LidarRanges[partitioned_arrays [i]]
                if tmpArray.size == 0:
                    continue    
                min_index=np.argmin(tmpArray)
                min_distances.append(min(tmpArray))
                min_distances_angles.append(angle_min+angle_increment*partitioned_arrays[i][min_index])
            # compute the coordinates of the obstacle point 0 in the fixed frame
            xo_min=[]
            yo_min=[]
            for i in range(len(min_distances)):
                xo_min.append(x+min_distances [i]*np.cos (min_distances_angles[i]+theta))
                yo_min.append(y+min_distances [i]*np. sin (min_distances_angles[i]+theta))
            
            # compute the gradient value for every obstacle
            g_values=[]
            gradUr=[]
            for i in range(len(min_distances)): 
                gradUr_1=np.array([[0],[0]])
                g_val=np.sqrt((x-xo_min[i])**2+(y-yo_min[i])**2)
                g_values.append(g_val)
                if (g_val<= gstar):
                    # scalar in front of expression
                    pr=kr*((1/gstar) - (1/g_values[i]))*(1/((g_values[i])**3))
                    gradUr_1=pr*np.array([[x-xo_min[i]], [y-yo_min[i]]])
                gradUr.append(gradUr_1)
            # compute the repulsive force
            RF=np.array([[0],[0]])
            for i in range(len(gradUr)):
                RF=RF+gradUr[i]
            RF=-RF
        if(obstacleYES):
            # total force
            F=AF+RF
        else:
            F=AF
        
        # desired orientation
        thetaD=math.atan2(F[1,0],F[0,0])

        # orientation angle error
        eorient=self.orientationError(theta,thetaD)

        #is distance is smaller than tolarance turn off the controller
        if (np.linalg.norm(vectorD)<self.eps_control):
            thetavel=0.0
            xvel=0.0
        else:
            #if orientation error is too large adjust the angle
            # angular velocity
            if (abs(eorient)>self.eps_orient):
                #adjust the orientation 
                thetavel=ktheta*eorient
                #no linear velocity
                xvel=0.2
            else:
                #if orientation error is enough or acceptable go towards the goal then adjust both
                #orientation
                thetavel=ktheta*eorient
                # linear velocity
                xvel=np.linalg.norm(F,2)

            if (np.abs(xvel)>2.6):
                xvel=2.5
        self.controlVel.linear.x=xvel
        self.controlVel.linear.y=0.0
        self.controlVel.linear.z=0.0
        self.controlVel.angular.x=0.0
        self.controlVel.angular.y=0.0
        self.controlVel.angular.z=thetavel

        print("sending the control command")
        self.ControlPublisher.publish(self.controlVel)

        timeDiff = self.msgOdometryTime - self.initialTime
        print(f"time, x, y, theta: ({timeDiff:.3f}, {x:.3f}, {y:.3f}, {theta:.3f})")
        print(f"xvel={xvel:.2f}, thetavel={thetavel:.2f}")

def main(args=None):
    rclpy.init(args=args)

    #desired point
    xdu=10
    ydu=-10
    #control parameters for attraction force
    kau=0.3
    #control parameters for repulsion force
    kru=10
    #control parameter for orientation controller
    kthetau=4
    #limiting influence of the repulsive function
    gstaru=4.0
    #tolarance for performing the first orientation adjustment and control
    eps_orientu=np.pi/10
    #turning off the controller when reached the goal
    eps_controlu=0.0

    TestNode = ControllerNode(xdu,ydu,kau,kru,kthetau,gstaru,eps_orientu,eps_controlu)

    rclpy.spin(TestNode)

    TestNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()