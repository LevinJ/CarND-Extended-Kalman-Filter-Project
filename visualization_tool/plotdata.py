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
        self.previous_timestamp = 0
        return
    def __load_input_data(self,kalman_input_file):
        
        
        self.previous_timestamp = 0
        entries = []
        cols = ['type_meas', 'px_meas','py_meas','vx_meas','vy_meas','timestampe','delta_time','px_gt','py_gt','vx_gt','vy_gt']
        with open(kalman_input_file, "r") as ins:
            for line in ins:
                entries.append(self.__process_line(line))
        df = pd.DataFrame(entries, columns=cols)
#         df.to_csv('input_data.csv')
        print('csv saved')
        return df
    def __cal_input_rmse(self,df):
        print('lidar only: {}'.format(self.__cal_rmse(df[df['type_meas'] == 'L'])))
        print('radar only: {}'.format(self.__cal_rmse(df[df['type_meas'] == 'R'])))
        print('all data: {}'.format(self.__cal_rmse(df)))
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
            type_meas,rho,phi,rho_dot, timestampe,px_gt,py_gt,vx_gt,vy_gt=line[:-1].split("\t")
            rho = float(rho)
            phi = float(phi)
            rho_dot = float(rho_dot)
            timestampe = int(timestampe)
            px_gt = float(px_gt)
            py_gt = float(py_gt)
            vx_gt = float(vx_gt)
            vy_gt = float(vy_gt)
            
            px_meas = rho * math.cos(phi)
            py_meas = rho * math.sin(phi)
            vx_meas = rho_dot * math.cos(phi)
            vy_meas = rho_dot * math.sin(phi)
        else:
            raise("unexpected line" + line)
        
        delta_time = 0
        if self.previous_timestamp != 0:
            delta_time = (timestampe - self.previous_timestamp)/1000000.0
        
        self.previous_timestamp = timestampe
            
        
        return type_meas, px_meas,py_meas,vx_meas,vy_meas,timestampe,delta_time, px_gt,py_gt,vx_gt,vy_gt
    def visulize_data(self,df):
        plt.scatter(df['px_gt'], df['py_gt'],marker='*', color='blue')
        plt.scatter(df['px_meas'], df['py_meas'],marker='v',color='red')
        plt.show()
        return
   
        
    def run(self):
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-1.txt'
        df = self.__load_input_data(kalman_input_file)
        self.__cal_input_rmse(df)
        self.visulize_data(df)

        return



if __name__ == "__main__":   
    obj= PlotData()
    obj.run()