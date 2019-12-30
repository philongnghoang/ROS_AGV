#!/usr/bin/env python
# import the necessary packages
from sklearn.cluster import DBSCAN
from sklearn.cluster import dbscan
#from imutils import build_montages
#from imutils import paths
import numpy as np
import argparse
#import pickle
import yaml
import os
import shutil

class Position_classify():
    def __init__(self):
        self.data_name = []
        self.data_pos = []
        self.data_qua = []
        self.noise = []
        self.all_goal = []
        self.eps_min = 0.1
        self.min_sample = 20
        with open("param_yaml/data_3.yaml", 'r') as stream:
            self.dataMap = yaml.load(stream)
        print(len(self.dataMap))
        self.data_process()
        self.classify_name()

    def classify_name(self):
        obj_in_map = self.get_label(self.data_name)
        print(obj_in_map)
        for label_ in obj_in_map:
            all_pos_obj = [self.data_pos[i] for i,n in enumerate(self.data_name) if n == label_]
            all_qua_obj = [self.data_qua[i] for i,n in enumerate(self.data_name) if n == label_]
            self.pos_qua_classify(label_,all_pos_obj,all_qua_obj)
        print(self.all_goal)
        with open('param_yaml/final_goal.yaml', 'w') as yaml_file:
            #for dt in self.all_goal:
            yaml.dump(self.all_goal, yaml_file, default_flow_style=False)

    def data_process(self):
        for obj in self.dataMap:
            #Name 
            self.data_name.append(obj['name_object'])
            #Position
            _pos = obj['position']
            self.data_pos.append([_pos['x'],_pos['y']])
            #Quaternion 
            _qua = obj['quaternion']
            self.data_qua.append([_qua['r3'],_qua['r4']])
        
    def pos_qua_classify(self,name_a_obj,pos_a_obj,qua_a_obj):
        print("[INFO] clustering...")
        clt = DBSCAN(algorithm='auto', 
                    eps=self.eps_min ,metric='euclidean', 
                    min_samples = self.min_sample, 
                    p=None) 
        clt.fit(pos_a_obj)
        labelIDs = np.unique(clt.labels_)
        print(labelIDs)
        numUnique = len(np.where(labelIDs > -1)[0])
        print("[INFO] Number of goals: {}".format(numUnique))
        
        for labelID in labelIDs:
            print("[INFO] faces for face ID: {}".format(labelID))

            idx = np.where(clt.labels_ == labelID)[0]
            # loop over the sampled indexes
            if str(labelID)=='-1':
                print("ID noise :",idx)
                self.calc_position(idx,labelID,name_a_obj,pos_a_obj,qua_a_obj)  
            else:
                #print("ID data :",idx)
                #self.all_goal.append([str(labelID),idx])
                self.all_goal.append(self.calc_position(idx,labelID,name_a_obj,pos_a_obj,qua_a_obj))
        
        #print("all goal",self.all_goal)
        #print("noise",self.noise)
        return self.all_goal    
    def calc_position(self,idex,label,nameaclass,posaclass,quaaclass):
        pos_in_class = []
        qua_in_class = []
        result_pos = dict()
        pos_ = dict()
        qua_ = dict()
        # Create a list positon and list
        pos_in_class = [posaclass[_id] for _id in idex]
        qua_in_class = [quaaclass[_id] for _id in idex]
        
        # Mean all positon (x,y) , quaternion (r3,r4)
        name_obj = nameaclass + "_" + str(label)
        mean_pos = np.mean(pos_in_class, axis=0)
        mean_qua = np.mean(qua_in_class, axis=0)
        
        #print("name",name_in_class)
        #print("pos in class",mean_pos)
        #print("qua in class",mean_qua)
        result_pos['name_object'] = name_obj

        pos_['x'] = float(round(mean_pos[0],3))
        pos_['y'] = float(round(mean_pos[1],3))
        result_pos['position'] = pos_

        qua_['r1'] = 0
        qua_['r2'] = 0
        qua_['r3'] = float(round(mean_qua[0],3))
        qua_['r4'] = float(round(mean_qua[1],3))
        result_pos['quaternion'] = qua_
        #print(result_pos)
        return result_pos
    def get_label(self,list_name):
        num_label = []
        for i in list_name:
            if i not in num_label:
                num_label.append(i)
        return num_label

if __name__ == '__main__':
    classify = Position_classify()
    #classify.pos_qua_classify()