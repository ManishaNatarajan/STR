
import numpy as np
from numpy.linalg import inv
from math import *  
from random import *

class sampling:

    def __init__(self, alpha1 =0.25, alpha2 = 0.25, alpha3 = 0.25 , alpha4 = 0.25):
        
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.alpha4 = alpha4
        return None

    def prior(self, map1):
        """
        Return one prior particle for injecting
        """
        Xt = []
        def fit_func():
            x = randint(0,799)
            y = randint(0,799)
            theta = np.random.uniform(-pi,pi)
            if((map1[y][x] >0.8)):
                return([x,y,theta])
                
            
            else:
                
                return fit_func()
                        
        

        return fit_func()


    def normalize(self, z):
        #To ensure angle is between [-pi, pi]
        return np.arctan2(sin(z), cos(z)) 

    def angle_diff(self,a,b):
        a = self.normalize(a)
        b = self.normalize(b)
        d1 = a-b
        d2 = 2*pi - fabs(d1)
        if(d1 > 0):
            d2 = d2*(-1.0)
        if(fabs(d1) < fabs(d2)):
            return d1
        else:
            return d2



    def sample_motion_model_odometry(self, ut,x_prev):
        
        """
        Input:
        
        ut is a list of x_bar_(t-1) and x_bar_t
        x_prev a list of x_prev
        
        Output:
        
        new state (list)
        """


        x_b = ut[0][0];
        y_b = ut[0][1];
        theta_b = ut[0][2];
        x_b_t = ut[1][0];
        y_b_t = ut[1][1];
        theta_b_t = ut[1][2];
    
        # Avoid computing bearing from two poses that are extremely close to one another- happens on in-place rotation
        if(np.sqrt((x_b - x_b_t)*(x_b - x_b_t)+(y_b - y_b_t)*(y_b - y_b_t)) < 0.01):
            delta_rot_1 = 0.0
        else:
            delta_rot_1 = self.angle_diff(np.arctan2(y_b_t - y_b, x_b_t - x_b), theta_b)
        delta_trans = np.sqrt((x_b - x_b_t)*(x_b - x_b_t)+(y_b - y_b_t)*(y_b - y_b_t))
        delta_rot_2 = self.angle_diff(theta_b_t - theta_b, delta_rot_1)
        #Treating backward and forward motion models symmetrically for the given below noise model:
        delta_rot_1_noise = min(fabs(self.angle_diff(delta_rot_1, 0)), fabs(self.angle_diff(delta_rot_1, pi)))
        delta_rot_2_noise = min(fabs(self.angle_diff(delta_rot_2, 0)), fabs(self.angle_diff(delta_rot_2, pi)))

        #Sample pose differernces:
        delta_rot_1_hat = self.angle_diff(delta_rot_1, np.random.normal(0, self.alpha1*delta_rot_1_noise*delta_rot_1_noise + self.alpha2*delta_trans*delta_trans))
        delta_trans_hat = delta_trans - np.random.normal(0, self.alpha3*delta_trans*delta_trans+self.alpha4*delta_rot_1_noise*delta_rot_1_noise+self.alpha4*delta_rot_2_noise*delta_rot_2_noise)
        delta_rot_2_hat = self.angle_diff(delta_rot_2, np.random.normal(0, self.alpha1*delta_rot_2_noise*delta_rot_2_noise + self.alpha2*delta_trans*delta_trans))

        #Apply sampled update to particle pose:
        x_t = x_prev[0] + delta_trans_hat*np.cos(x_prev[2] + delta_rot_1_hat)
        y_t = x_prev[1] + delta_trans_hat*np.sin(x_prev[2] + delta_rot_1_hat)
        theta_t = x_prev[2] + delta_rot_1_hat + delta_rot_2_hat

        return [x_t, y_t, theta_t]

        # del_rot_1 = np.arctan2(y_b_t - y_b, x_b_t - x_b) - theta_b
        # del_trans = np.sqrt(np.square(x_b - x_b_t) + np.square(y_b - y_b_t))
        # del_rot_2 = theta_b_t - theta_b - del_rot_1
        
        # del_rot_1_op = del_rot_1 - np.random.normal(0,np.sqrt(self.alpha1*abs(del_rot_1) + self.alpha2*abs(del_trans)))
        # del_trans_op = del_trans - np.random.normal(0,np.sqrt(self.alpha3*abs(del_trans) + self.alpha4*abs(del_rot_1 + del_rot_2)) )
        # del_rot_2_op = del_rot_2 - np.random.normal(0,np.sqrt(self.alpha1*abs(del_rot_2) + self.alpha2*abs(del_trans)))


        # diff1 = del_rot_1 - del_rot_1_op
        # diff2 = del_rot_2 - del_rot_2_op 

        
        # # if(diff1 > pi):
        # #     if((diff1 -  ((diff1)//2*pi)*2*pi) <=pi):
        # #        diff1 =  (diff1 -  ((diff1)//2*pi)*2*pi)
            
        # #     elif((diff1 - ((diff1)//2*pi)*2*pi) > pi):
        # #         diff1 =  (diff1 -  (((diff1)//2*pi)+1)*2*pi)
        
        # # if(diff2 > pi):
        # #     if((diff2 -  ((diff2)//2*pi)*2*pi) <=pi):
        # #        diff2 =  (diff2 -  ((diff2)//2*pi)*2*pi)
            
        # #     elif((diff2 - ((diff2)//2*pi)*2*pi) > pi):
        # #         diff2 =  (diff2 -  (((diff2)//2*pi)+1)*2*pi)
                
        
        # del_rot_1_op = del_rot_1 - diff1
        # del_rot_2_op = del_rot_2 - diff2
        
        # x_t = x_prev[0] + del_trans_op*np.cos(x_prev[2] + del_rot_1_op)
        # y_t = x_prev[1] + del_trans_op*np.sin(x_prev[2] + del_rot_1_op)
        # theta_t = x_prev[2] + del_rot_1_op + del_rot_2_op

        # return [x_t, y_t, theta_t]


    def sample_motion_model_with_map(self,ut, xt_1, m):
        """
        xt_1: list of particles


        """
        particle = 0
        x_t = []


        while(particle!=len(xt_1)):
            x = self.sample_motion_model_odometry(ut,xt_1[particle])
            #Randomly generate a new particle if the particle goes out of map
            if((int(floor(x[1])) <0 ) or (int(floor(x[1])) > 799) or (int(floor(x[0])) < 0) or (int(floor(x[0])) > 799)):

                x = self.prior(m)

            
            #Randomly generate a new particle if the particle hits the wall
            elif((m[int(floor(x[1])),int(floor(x[0]))] ==0)):
                x = self.prior(m)


            x_t.append(x)


            particle+=1

        return x_t
