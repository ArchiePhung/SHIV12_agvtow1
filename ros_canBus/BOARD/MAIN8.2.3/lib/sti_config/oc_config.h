#ifndef STI_CONFIG_H
#define STI_CONFIG_H

// #define E1 19 //1
#define ENB_LIFTER 	14 	// 	1
#define LIFTER_HG 	26 	//	1 or 0 26
#define LIFTER_LW  	27 	//	0 or 1 27
#define SS_UP 		13 	// 	CB1 
#define SS_DOWN 	18 	// 	CB5
#define SS_LIFT		12	// 	CB2
// CB1: 13 | CB2: 12 | CB3: 15 | CB4: 18 | CB5: 21 | CB6: 22 | CB7 :23 | E1: 19 |
#define TIME_CHECK_ERROR 40000 // ms
//--------------- HZ check ---------------//
#define PUB_STATUS_FREQUENCY 10 // hz tần số pub dữ liệu trạng thái Main - Max: 20
#define CHECK_CONNECT_PC_FREQUENCY 2		// hz tần số kiểm tra kết nói giữa OC vs PC.

#endif /*AGV_CONTROLLER_CONFIG_H*/
