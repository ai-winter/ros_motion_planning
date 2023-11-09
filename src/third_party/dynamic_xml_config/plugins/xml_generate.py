#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.2                                                                       *
*  @date     2023.04.23                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import os
import sys
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod

import yaml


class XMLGenerator(ABC):
    def __init__(self) -> None:
        # get the root path of package(src layer)
        self.root_path = os.path.split(os.path.realpath(__file__))[0] + "/../../../"
        # user configure
        self.user_cfg = XMLGenerator.yamlParser(self.root_path + "user_config/" + sys.argv[1])

    @abstractmethod
    def plugin(self):
        """
        [interface] Implement of specific application.

        Return
        ----------
        app_register: list of ET.Element to register
        """
        pass

    @staticmethod
    def yamlParser(path: str) -> dict:
        """
        Parser user configure file(.yaml).

        Parameters
        ----------
        path: str
            the path of file(.yaml).

        Return
        ----------
        data: dict
            user configuer tree
        """
        with open(path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        return data

    @staticmethod
    def indent(elem: ET.Element, level: int = 0) -> None:
        """
        Format the generated xml document.

        Parameters
        ----------
        elem: ET.Element
        level: int
            indent level.
        """
        i = "\n" + level * "\t"
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "\t"
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                XMLGenerator.indent(elem, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    @staticmethod
    def createElement(name: str, text: str = None, props: dict = {}) -> ET.Element:
        e = ET.Element(name, attrib=props)
        if not text is None:
            e.text = text
        return e
