import sys
import os
# from _pickle import dump
sys.path.insert(0, os.path.abspath('..'))

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math


class PlotData(object):
    def __init__(self):
       
        return
    def __load_input_data(self,kalman_input_file):
        
        print('processing {}.....'.format(kalman_input_file))
        self.previous_timestamp = None
        self.previous_vx = None
        self.previous_vy = None
        
        entries = []
        cols = ['type_meas', 'px_meas','py_meas','vx_meas','vy_meas','timestampe','delta_time','px_gt','py_gt','vx_gt','vy_gt',
                'rho_meas','phi_meas','rho_dot_meas', 'rho_gt','phi_gt','rho_dot_gt','ax','ay']
        with open(kalman_input_file, "r") as ins:
            for line in ins:
                entries.append(self.__process_line(line))
        df = pd.DataFrame(entries, columns=cols)
        csv_file_name = os.path.dirname(kalman_input_file) + '/preprocess/' + os.path.basename(kalman_input_file)[:-4] +'.csv'
        df.to_csv(csv_file_name)
        print(csv_file_name + ' saved')
        return df
    def __cal_input_rmse(self,df):
        lidar_df = df[df['type_meas'] == 'L']
        radar_df = df[df['type_meas'] == 'R']
        print('lidar only: {}'.format(self.__cal_rmse(lidar_df)))
        print('radar only: {}'.format(self.__cal_rmse(radar_df)))
        print('all data: {}'.format(self.__cal_rmse(df)))
        
        print("lidar measurement noise {}, {}".format(np.var(lidar_df['px_meas'] - lidar_df['px_gt']), np.var(lidar_df['py_meas'] - lidar_df['py_gt'])))
        print("radar measurement noise {}, {}, {}".format(np.var(radar_df['rho_meas'] - radar_df['rho_gt']), np.var(radar_df['phi_meas'] - radar_df['phi_gt']),
                                                 np.var(radar_df['rho_dot_meas'] - radar_df['rho_dot_gt'])))
        print("motion noise {}, {}".format(np.var(df[df['ax']!=0]['ax']), np.var(df[df['ay']!=0]['ay'])))
        
        return
    def __cal_rmse(self, df):
        px_rmse = math.sqrt(((df['px_meas'] - df['px_gt']).values ** 2).mean())
        py_rmse = math.sqrt(((df['py_meas'] - df['py_gt']).values ** 2).mean())
        vx_rmse = math.sqrt(((df['vx_meas'] - df['vx_gt']).values ** 2).mean())
        vy_rmse = math.sqrt(((df['vy_meas'] - df['vy_gt']).values ** 2).mean())
        
        
        return px_rmse,py_rmse,vx_rmse,vy_rmse
    def __process_line(self, line):
        px_meas = 0
        py_meas = 0
        vx_meas = 0
        vy_meas = 0
        timestampe = 0
        px_gt = 0
        py_gt = 0
        vx_gt = 0
        vy_gt = 0
        
        rho_gt =0
        phi_gt = 0
        rho_dot_gt = 0
        rho_meas = 0
        phi_meas = 0
        rho_dot_meas = 0
        
        if 'L'in line:
            type_meas,px_meas,py_meas, timestampe,px_gt,py_gt,vx_gt,vy_gt=line[:-1].split("\t")
            px_meas = float(px_meas)
            py_meas = float(py_meas)
            timestampe = int(timestampe)
            px_gt = float(px_gt)
            py_gt = float(py_gt)
            vx_gt = float(vx_gt)
            vy_gt = float(vy_gt)
        elif 'R' in line:
            type_meas,rho_meas,phi_meas,rho_dot_meas, timestampe,px_gt,py_gt,vx_gt,vy_gt=line[:-1].split("\t")
            rho_meas = float(rho_meas)
            phi_meas = float(phi_meas)
            rho_dot_meas = float(rho_dot_meas)
            timestampe = int(timestampe)
            px_gt = float(px_gt)
            py_gt = float(py_gt)
            vx_gt = float(vx_gt)
            vy_gt = float(vy_gt)
            
            px_meas = rho_meas * math.cos(phi_meas)
            py_meas = rho_meas * math.sin(phi_meas)
            vx_meas = rho_dot_meas * math.cos(phi_meas)
            vy_meas = rho_dot_meas * math.sin(phi_meas)
            
            rho_gt = math.sqrt(px_gt ** 2 + py_gt ** 2)
            if px_gt != 0:
                phi_gt = math.atan(py_gt/px_gt)
            else:
                print('px_gt is zero')
            
            if rho_gt != 0:
                rho_dot_gt = (px_gt * vx_gt + py_gt * vy_gt) / rho_gt
            else:
                print('rho_gt is zero')
        else:
            raise("unexpected line" + line)
        
        delta_time = 0
        ax = 0
        ay = 0
        if self.previous_timestamp is not None:
            delta_time = (timestampe - self.previous_timestamp)/1000000.0
        
        if delta_time != 0:    
            if self.previous_vx is not None:
                ax = (vx_gt - self.previous_vx)/ delta_time
            if self.previous_vy is not None:
                ay = (vy_gt - self.previous_vy)/ delta_time
            
        
        self.previous_timestamp = timestampe
        self.previous_vx = vx_gt
        self.previous_vy = vy_gt
            
        
        return type_meas, px_meas,py_meas,vx_meas,vy_meas,timestampe,delta_time, px_gt,py_gt,vx_gt,vy_gt,\
            rho_meas,phi_meas,rho_dot_meas, rho_gt,phi_gt,rho_dot_gt,ax,ay
    def visulize_data(self,df):
        plt.scatter(df['px_gt'], df['py_gt'],marker='*', color='blue')
        plt.scatter(df['px_meas'], df['py_meas'],marker='v',color='red')
        plt.show()
        return
    def run_data_1(self):
        print('####data sample 1###')
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-1.txt'
        df = self.__load_input_data(kalman_input_file)
        self.__cal_input_rmse(df)
        return df
    def run_data_2(self):
        print('####data sample 2###')
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-2.txt'
        df = self.__load_input_data(kalman_input_file)
        self.__cal_input_rmse(df)
        return df
    
   
        
    def run(self):
        self.run_data_1()
        self.run_data_2()
        
#         self.visulize_data(df)

        return



if __name__ == "__main__":   
    obj= PlotData()
    obj.run()