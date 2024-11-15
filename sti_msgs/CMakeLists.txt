cmake_minimum_required(VERSION 2.8.3)
project(sti_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
	DIRECTORY msg
	FILES
	Velocities.msg
	EMC.msg
	Modbus_0x.msg
	Modbus_4x_read.msg
	Modbus_4x_write.msg

	Goal_control.msg
	Status_goalControl.msg
	Status_goal_control.msg
	Parking_status.msg
	Parking_control.msg
	Setpose_control.msg
	Setpose_status.msg
	Safety_zone.msg
	Mission_status.msg
	Mission_control.msg 
	#new
	Client_request.msg
	Client_respond.msg
	Move_request.msg
	Move_respond.msg 
	Ps2_msgs.msg
	Read_prepheral.msg
	Write_prepheral.msg
	Status_base.msg
	Motor_parameter.msg
	Motor_rpm.msg
	
	Check_shelves.msg


	FL_infoRespond.msg
	FL_infoRequest.msg
	FL_cmdRespond.msg
	FL_cmdRequest.msg  
	NN_cmdRequest.msg 
	NN_infoRespond.msg
	NN_infoRequest.msg

	HMI_papeAuto.msg
	HMI_papeByHand.msg
	HMI_papeLaunch.msg
	HMI_papeDetailFL.msg
	HMI_papeDetailNN.msg
	HMI_papeMessage.msg
	HMI_allButton.msg
	HMI_infoGeneral.msg
	HMI_buttonPassword.msg
	HMI_papePassword.msg

	PID.msg
	Main_missionControl.msg
	Main_status.msg
	Main_switchMode.msg
	Zone_lidar_2head.msg
	
	ManageLaunch.msg
	Port_status.msg
	Reconnect_status.msg
	Reconnect_stt.msg

	Magnetic_line.msg
	RFID.msg

	#hc
	HC_info.msg
	HC_request.msg
	#main
	POWER_info.msg
	POWER_request.msg
	Lift_control.msg	
	Lift_status.msg
	#oc
	Lift_control.msg
	Lift_status.msg
	MC_status.msg

	#test
	Tf_lms.msg

	# new parking
	APathParking.msg
	PointOfPath.msg

	#navi new
	PathInfo.msg
	PointRequestMove.msg
	ListPointRequestMove.msg
	LineRequestMove.msg
	StopPoint.msg

	Navi_cmdRequest.msg

	# new sound
	Sound_request.msg
    Try_query.msg

	Nuc_info.msg
        EncoderData.msg
)	

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
