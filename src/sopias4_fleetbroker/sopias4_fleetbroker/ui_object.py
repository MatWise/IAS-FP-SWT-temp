# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/sopias4_fleetbroker/assets/frontend/gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1580, 650)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_conf_init = QtWidgets.QWidget()
        self.tab_conf_init.setObjectName("tab_conf_init")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.tab_conf_init)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label = QtWidgets.QLabel(self.tab_conf_init)
        self.label.setTextFormat(QtCore.Qt.MarkdownText)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout_2.addWidget(self.label)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_5 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_5.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_4.addWidget(self.label_5)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.checkBox_use_respawn = QtWidgets.QCheckBox(self.tab_conf_init)
        self.checkBox_use_respawn.setObjectName("checkBox_use_respawn")
        self.horizontalLayout_4.addWidget(self.checkBox_use_respawn)
        self.checkBox_use_autostart = QtWidgets.QCheckBox(self.tab_conf_init)
        self.checkBox_use_autostart.setObjectName("checkBox_use_autostart")
        self.horizontalLayout_4.addWidget(self.checkBox_use_autostart)
        self.checkBox_domain_bridge = QtWidgets.QCheckBox(self.tab_conf_init)
        self.checkBox_domain_bridge.setObjectName("checkBox_domain_bridge")
        self.horizontalLayout_4.addWidget(self.checkBox_domain_bridge)
        self.horizontalLayout.addLayout(self.horizontalLayout_4)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.lineEdit_path_params_file = QtWidgets.QLineEdit(self.tab_conf_init)
        self.lineEdit_path_params_file.setObjectName("lineEdit_path_params_file")
        self.horizontalLayout_5.addWidget(self.lineEdit_path_params_file)
        self.pushButton_pick_params_file = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_pick_params_file.setObjectName("pushButton_pick_params_file")
        self.horizontalLayout_5.addWidget(self.pushButton_pick_params_file)
        self.verticalLayout_4.addLayout(self.horizontalLayout_5)
        self.label_8 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_8.setObjectName("label_8")
        self.verticalLayout_4.addWidget(self.label_8)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pushButton_bringup_server = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_bringup_server.setObjectName("pushButton_bringup_server")
        self.horizontalLayout_2.addWidget(self.pushButton_bringup_server)
        self.pushButton_stop_map_server = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_stop_map_server.setObjectName("pushButton_stop_map_server")
        self.horizontalLayout_2.addWidget(self.pushButton_stop_map_server)
        self.verticalLayout_4.addLayout(self.horizontalLayout_2)
        self.label_6 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_6.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_4.addWidget(self.label_6)
        self.label_7 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_7.setObjectName("label_7")
        self.verticalLayout_4.addWidget(self.label_7)
        self.gridLayout_6 = QtWidgets.QGridLayout()
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.pushButton_launch_map_server = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_launch_map_server.setObjectName("pushButton_launch_map_server")
        self.gridLayout_6.addWidget(self.pushButton_launch_map_server, 1, 1, 1, 1)
        self.pushButton_launch_mrv = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_launch_mrv.setObjectName("pushButton_launch_mrv")
        self.gridLayout_6.addWidget(self.pushButton_launch_mrv, 1, 0, 1, 1)
        self.pushButton_domain_bridge = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_domain_bridge.setObjectName("pushButton_domain_bridge")
        self.gridLayout_6.addWidget(self.pushButton_domain_bridge, 1, 2, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_6)
        self.verticalLayout_2.addLayout(self.verticalLayout_4)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        self.tabWidget.addTab(self.tab_conf_init, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.tab_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_14 = QtWidgets.QLabel(self.tab_2)
        self.label_14.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.verticalLayout_6.addWidget(self.label_14)
        self.label_10 = QtWidgets.QLabel(self.tab_2)
        self.label_10.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_10.setObjectName("label_10")
        self.verticalLayout_6.addWidget(self.label_10)
        self.tableWidget_registered_robots = QtWidgets.QTableWidget(self.tab_2)
        self.tableWidget_registered_robots.setObjectName("tableWidget_registered_robots")
        self.tableWidget_registered_robots.setColumnCount(5)
        self.tableWidget_registered_robots.setRowCount(0)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_registered_robots.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_registered_robots.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_registered_robots.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_registered_robots.setHorizontalHeaderItem(3, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_registered_robots.setHorizontalHeaderItem(4, item)
        self.tableWidget_registered_robots.horizontalHeader().setCascadingSectionResizes(True)
        self.tableWidget_registered_robots.horizontalHeader().setDefaultSectionSize(100)
        self.tableWidget_registered_robots.horizontalHeader().setStretchLastSection(True)
        self.tableWidget_registered_robots.verticalHeader().setCascadingSectionResizes(False)
        self.tableWidget_registered_robots.verticalHeader().setHighlightSections(True)
        self.tableWidget_registered_robots.verticalHeader().setStretchLastSection(False)
        self.verticalLayout_6.addWidget(self.tableWidget_registered_robots)
        self.verticalLayout_3.addLayout(self.verticalLayout_6)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_2 = QtWidgets.QLabel(self.tab_2)
        self.label_2.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_6.addWidget(self.label_2)
        self.lineEdit_unregister = QtWidgets.QLineEdit(self.tab_2)
        self.lineEdit_unregister.setObjectName("lineEdit_unregister")
        self.horizontalLayout_6.addWidget(self.lineEdit_unregister)
        self.pushButton_unregister = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_unregister.setObjectName("pushButton_unregister")
        self.horizontalLayout_6.addWidget(self.pushButton_unregister)
        self.verticalLayout_3.addLayout(self.horizontalLayout_6)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_9 = QtWidgets.QLabel(self.tab_2)
        self.label_9.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_9.setObjectName("label_9")
        self.verticalLayout_5.addWidget(self.label_9)
        self.textEdit_logger = QtWidgets.QTextEdit(self.tab_2)
        self.textEdit_logger.setObjectName("textEdit_logger")
        self.verticalLayout_5.addWidget(self.textEdit_logger)
        self.verticalLayout_3.addLayout(self.verticalLayout_5)
        self.tabWidget.addTab(self.tab_2, "")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.verticalLayout_10 = QtWidgets.QVBoxLayout(self.tab)
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.label_19 = QtWidgets.QLabel(self.tab)
        self.label_19.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.verticalLayout_10.addWidget(self.label_19)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.label_20 = QtWidgets.QLabel(self.tab)
        self.label_20.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_20.setObjectName("label_20")
        self.verticalLayout_8.addWidget(self.label_20)
        self.label_21 = QtWidgets.QLabel(self.tab)
        self.label_21.setObjectName("label_21")
        self.verticalLayout_8.addWidget(self.label_21)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.lineEdit = QtWidgets.QLineEdit(self.tab)
        self.lineEdit.setObjectName("lineEdit")
        self.horizontalLayout_3.addWidget(self.lineEdit)
        self.pushButton_load_map = QtWidgets.QPushButton(self.tab)
        self.pushButton_load_map.setObjectName("pushButton_load_map")
        self.horizontalLayout_3.addWidget(self.pushButton_load_map)
        self.verticalLayout_8.addLayout(self.horizontalLayout_3)
        self.verticalLayout_10.addLayout(self.verticalLayout_8)
        self.verticalLayout_9 = QtWidgets.QVBoxLayout()
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.label_22 = QtWidgets.QLabel(self.tab)
        self.label_22.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_22.setObjectName("label_22")
        self.verticalLayout_9.addWidget(self.label_22)
        self.label_23 = QtWidgets.QLabel(self.tab)
        self.label_23.setObjectName("label_23")
        self.verticalLayout_9.addWidget(self.label_23)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.doubleSpinBox_occupied_thres = QtWidgets.QDoubleSpinBox(self.tab)
        self.doubleSpinBox_occupied_thres.setMaximum(1.0)
        self.doubleSpinBox_occupied_thres.setSingleStep(0.01)
        self.doubleSpinBox_occupied_thres.setObjectName("doubleSpinBox_occupied_thres")
        self.gridLayout_4.addWidget(self.doubleSpinBox_occupied_thres, 3, 1, 1, 1)
        self.lineEdit_map_name = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_map_name.setObjectName("lineEdit_map_name")
        self.gridLayout_4.addWidget(self.lineEdit_map_name, 0, 1, 1, 1)
        self.lineEdit_map_topic = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_map_topic.setObjectName("lineEdit_map_topic")
        self.gridLayout_4.addWidget(self.lineEdit_map_topic, 0, 3, 1, 1)
        self.label_25 = QtWidgets.QLabel(self.tab)
        self.label_25.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_25.setObjectName("label_25")
        self.gridLayout_4.addWidget(self.label_25, 1, 0, 1, 1)
        self.label_29 = QtWidgets.QLabel(self.tab)
        self.label_29.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_29.setObjectName("label_29")
        self.gridLayout_4.addWidget(self.label_29, 3, 2, 1, 1)
        self.doubleSpinBox_free_thres = QtWidgets.QDoubleSpinBox(self.tab)
        self.doubleSpinBox_free_thres.setMaximum(1.0)
        self.doubleSpinBox_free_thres.setSingleStep(0.01)
        self.doubleSpinBox_free_thres.setObjectName("doubleSpinBox_free_thres")
        self.gridLayout_4.addWidget(self.doubleSpinBox_free_thres, 3, 3, 1, 1)
        self.label_27 = QtWidgets.QLabel(self.tab)
        self.label_27.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_27.setObjectName("label_27")
        self.gridLayout_4.addWidget(self.label_27, 0, 2, 1, 1)
        self.label_24 = QtWidgets.QLabel(self.tab)
        self.label_24.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_24.setObjectName("label_24")
        self.gridLayout_4.addWidget(self.label_24, 0, 0, 1, 1)
        self.comboBox_map_mode = QtWidgets.QComboBox(self.tab)
        self.comboBox_map_mode.setObjectName("comboBox_map_mode")
        self.gridLayout_4.addWidget(self.comboBox_map_mode, 1, 1, 1, 1)
        self.label_28 = QtWidgets.QLabel(self.tab)
        self.label_28.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_28.setObjectName("label_28")
        self.gridLayout_4.addWidget(self.label_28, 3, 0, 1, 1)
        self.label_26 = QtWidgets.QLabel(self.tab)
        self.label_26.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_26.setObjectName("label_26")
        self.gridLayout_4.addWidget(self.label_26, 1, 2, 1, 1)
        self.comboBox_image_format = QtWidgets.QComboBox(self.tab)
        self.comboBox_image_format.setObjectName("comboBox_image_format")
        self.gridLayout_4.addWidget(self.comboBox_image_format, 1, 3, 1, 1)
        self.pushButton_save_map = QtWidgets.QPushButton(self.tab)
        self.pushButton_save_map.setObjectName("pushButton_save_map")
        self.gridLayout_4.addWidget(self.pushButton_save_map, 4, 3, 1, 1)
        self.verticalLayout_9.addLayout(self.gridLayout_4)
        self.verticalLayout_10.addLayout(self.verticalLayout_9)
        spacerItem1 = QtWidgets.QSpacerItem(20, 446, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_10.addItem(spacerItem1)
        self.tabWidget.addTab(self.tab, "")
        self.verticalLayout.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Sopias4 Fleetbroker"))
        self.label.setText(_translate("MainWindow", "# Initialization and Configuration"))
        self.label_5.setText(_translate("MainWindow", "## Manage Turtlebot"))
        self.checkBox_use_respawn.setText(_translate("MainWindow", "Use respawn"))
        self.checkBox_use_autostart.setText(_translate("MainWindow", "Autostart components"))
        self.checkBox_domain_bridge.setText(_translate("MainWindow", "Use Sopias4 domain bridge"))
        self.pushButton_pick_params_file.setText(_translate("MainWindow", "Change configuration parameter file"))
        self.label_8.setText(_translate("MainWindow", "Before launching the Turtlebot, make sure the name space is registered. If using the simulation, then make sure to set the namespace also inside gazebo"))
        self.pushButton_bringup_server.setText(_translate("MainWindow", "Launch Sopias4 Map-Server"))
        self.pushButton_stop_map_server.setText(_translate("MainWindow", "Stop Sopias4 Map-Server"))
        self.label_6.setText(_translate("MainWindow", "### Manual launch sub-components"))
        self.label_7.setText(_translate("MainWindow", "By launching sub-components individually, the \"Stop Sopias4 Map-Server\" button doesn\'t work as intended."))
        self.pushButton_launch_map_server.setText(_translate("MainWindow", "Launch Map-Server"))
        self.pushButton_launch_mrv.setText(_translate("MainWindow", "Launch Multi Robot Coordinator"))
        self.pushButton_domain_bridge.setText(_translate("MainWindow", "Launch Sopias4 Domain Bridge"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_conf_init), _translate("MainWindow", "Initialization/Configuration"))
        self.label_14.setText(_translate("MainWindow", "# Monitoring"))
        self.label_10.setText(_translate("MainWindow", "## Registered Turtlebots"))
        item = self.tableWidget_registered_robots.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Namespace"))
        item = self.tableWidget_registered_robots.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Current pose"))
        item = self.tableWidget_registered_robots.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Current navigation goal"))
        item = self.tableWidget_registered_robots.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "Is navigating"))
        item = self.tableWidget_registered_robots.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "Current velocity"))
        self.label_2.setText(_translate("MainWindow", "**Unregister namespace:**"))
        self.pushButton_unregister.setText(_translate("MainWindow", "Unregister"))
        self.label_9.setText(_translate("MainWindow", "## Logging"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Monitoring"))
        self.label_19.setText(_translate("MainWindow", "# Mapping"))
        self.label_20.setText(_translate("MainWindow", "## Operating"))
        self.label_21.setText(_translate("MainWindow", "Starting and stopping the mapping process. When stopped, the map is saved on the Sopias4 Map-Server with the confiugation settings below"))
        self.pushButton_load_map.setText(_translate("MainWindow", "Load map"))
        self.label_22.setText(_translate("MainWindow", "## Configuration for saving map"))
        self.label_23.setText(_translate("MainWindow", "Configure the parameters with which the map is saved on the Sopias4 Map-Server. The default configuration is the map which is automatically loaded when launchong Sopias4 Map-Server without passing parameters"))
        self.label_25.setText(_translate("MainWindow", "**Map mode:**"))
        self.label_29.setText(_translate("MainWindow", "**Free threshold:**"))
        self.label_27.setText(_translate("MainWindow", "**Map topic:**"))
        self.label_24.setText(_translate("MainWindow", "**Map path:**"))
        self.label_28.setText(_translate("MainWindow", "**Threshold occupied:**"))
        self.label_26.setText(_translate("MainWindow", "**Image format:**"))
        self.pushButton_save_map.setText(_translate("MainWindow", "Save map"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Maps"))