#!/usr/bin/python
#-*- coding: utf-8 -*-

''' 
******************************************************************************************
*  Copyright (C) 2022 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    rviz basic pannel configure class.                                          *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
'''
import yaml

class RVizConfig:
    def __init__(self, rvizName, path, baseFile=None):
        # Rviz可视化管理器
        self.VIZ_MAN = 'Visualization Manager'
        # Rviz标准工具集
        self.STD_TOOLS = ['rviz/Interact', 'rviz/MoveCamera', 'rviz/Select', 'rviz/FocusCamera', 'rviz/Measure', 'rviz/PublishPoint']
        # Rviz标准面板 
        self.STD_PANELS = ['rviz/Displays', 'rviz/Tool Properties']  
        # Rviz文件名
        self.fileName = rvizName
        # Rviz缓存路径
        self.cachePath = path
        # Rviz配置数据
        self.data = yaml.load(baseFile) if baseFile else {}

    def __repr__(self):
        return yaml.dump(self.data, default_flow_style=False)

    # @breif: 设置Rviz配置路径
    # @param[in]: path  Rviz配置路径
    # @return: None
    def setFileCache(self, path):
        self.cachePath = path

    # @breif: 获得rviz Visualization Manager的全部配置数据
    # @param[in]: None
    # @return: rviz Visualization Manager的全部配置数据
    def getVisualization(self):
        if self.VIZ_MAN not in self.data:
            self.data[self.VIZ_MAN] = {}
        return self.data[self.VIZ_MAN]            

    # @breif: 设置模型显示
    # @param[in]: displays  显示格式
    # @return: True->设置成功 False->设置失败，请先调用清除模型显示
    def setDisplays(self, displays):
        if 'Displays' not in self.getVisualization():
            self.data[self.VIZ_MAN]['Displays'] = displays
            print("Displays have been set successfully!")
            return True
        print("Displays setting failed! please call displays delete first!")
        return False

    # @breif: 清除模型显示
    # @param[in]: None
    # @return: True->清除成功 False->清除失败，请先调用设置模型显示
    def delDisplays(self):
        if 'Displays'  in self.getVisualization():
            del self.data[self.VIZ_MAN]['Displays']
            print("Displays have been deleted successfully!")
            return True
        print("Displays delete failed! please call displays setting first!")
        return False  

    # @breif: 设置Rviz标准工具集
    # @param[in]: None
    # @return: True->设置成功 False->设置失败，请先调用清除工具集    
    def setStdTools(self):
        v = self.getVisualization()
        if 'Tools' not in v:
            v['Tools'] = []
            for tool in self.STD_TOOLS:
                v['Tools'].append({'Class': tool})
            print("standard tools have been set successfully!")
            return True
        print("standard tools setting failed! please call standard tools delete first!")
        return False

    # @breif: 清除Rviz标准工具集
    # @param[in]: None
    # @return: True->清除成功 False->清除失败，请先调用设置Rviz标准工具集
    def delStdTools(self):
        if 'Tools'  in self.getVisualization():
            del self.data[self.VIZ_MAN]['Tools']
            print("standard tools have been deleted successfully!")
            return True
        print("standard tools delete failed! please call standard tools setting first!")
        return False  

    # @breif: 设置Rviz标准面板
    # @param[in]: None
    # @return: True->设置成功 False->设置失败，请先调用清除面板 
    def setStdPanels(self):
        if 'Panels' not in self.data:
            self.data['Panels'] = []
            for tool in self.STD_PANELS:
                self.data['Panels'].append({'Class': tool, 'Name': tool.split('/')[-1]})
                print("standard panels have been set successfully!")
            return True
        print("standard panels setting failed! please call standard panels delete first!")
        return False

    # @breif: 清除Rviz标准面板
    # @param[in]: None
    # @return: True->清除成功 False->清除失败，请先调用设置Rviz标准工具集
    def delStdPanels(self):
        if 'Panels'  in self.data:
            del self.data['Panels']
            print("standard panels have been deleted successfully!")
            return True
        print("standard panels delete failed! please call standard panels setting first!")
        return False 

    # @breif: 设置基坐标系
    # @param[in]: frame 基坐标系，默认为map
    # @return: None        
    def setFixedFrame(self, frame='map'):
        v = self.getVisualization()
        if 'Global Options' not in v:
            v['Global Options'] = {'Fixed Frame':frame}
        else:
            v['Global Options']['Fixed Frame'] = frame

    # @breif: 设置Rviz标准工具的话题
    # @param[in]: name  Rviz标准工具类
    # @param[in]: topic  Rviz标准工具对应的话题
    # @return: None
    def setToolTopic(self, name, topic):
        for m in self.data[self.VIZ_MAN]['Tools']:
            if m.get('Class', '')==name:
                m['Topic'] = topic

    # @breif: 添加目标导航工具及其话题
    # @param[in]: None
    # @return: None  
    def appendToolGoal(self, topic):
        v = self.getVisualization()
        if 'Tools' not in v:
            v['Tools'] = []
        v['Tools'].append({'Class': 'rviz/SetGoal', 'Topic': topic})

    # @breif: 添加初始位置设置及其话题
    # @param[in]: None
    # @return: None 
    def appendToolInit(self, topic):
        v = self.getVisualization()
        if 'Tools' not in v:
            v['Tools'] = []
        v['Tools'].append({'Class': 'rviz/SetInitialPose', 'Topic': topic})
        
    # @breif: 启动Rviz并应用用户配置
    # @param[in]: None
    # @return: None  
    def run(self):
        with open( self.cachePath + self.fileName + '.rviz', "w") as fp:
            fp.write(str(self))
        import subprocess
        subprocess.call(['rosrun','rviz','rviz', '-d', self.cachePath + self.fileName + '.rviz'])