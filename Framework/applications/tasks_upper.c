#include "tasks_upper.h"
#include "drivers_uartupper_user.h"

#include "drivers_flash.h"

#include "utilities_debug.h"

NaiveIOPoolDefine(upperIOPool, {0});

extern uint16_t yawAngle, pitchAngle;
int forPidDebug = 0;

extern int8_t flUpDown, frUpDown, blUpDown, brUpDown, allUpDown;
//extern float yawAngleTarget, pitchAngleTarget;
void getCtrlUartTask(void const * argument){
	uint8_t data[10];
	while(1){
		if(IOPool_hasNextRead(ctrlUartIOPool, 0)){
			IOPool_getNextRead(ctrlUartIOPool, 0);
			
			uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;
			for(uint8_t i = 0; i != 10; ++i){
				data[i] = pData[i];
			}			
			
//			int16_t yawZeroAngle = 1075, pitchZeroAngle = 710;
//			float yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8191.0;
//			yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
//			yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
//			float pitchRealAngle = (pitchZeroAngle - pitchAngle) * 360 / 8191.0;
//			pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
//			pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
			
			IOPool_pGetWriteData(upperIOPool)->yawAdd = 0;
			IOPool_pGetWriteData(upperIOPool)->pitchAdd = 0;
			if(data[0] == 'L'){
				IOPool_pGetWriteData(upperIOPool)->yawAdd = + 1.0f * ((int)data[1] - '0');
				//yawAngleTarget = yawRealAngle + 1.0f * ((int)data[1] - 48);
			}else if(data[0] == 'R'){
				IOPool_pGetWriteData(upperIOPool)->yawAdd = - 1.0f * ((int)data[1] - '0');
				//yawAngleTarget = yawRealAngle - 1.0f * ((int)data[1] - 48);
			}else if(data[0] == 'U'){
				IOPool_pGetWriteData(upperIOPool)->pitchAdd = + 8.0f * ((int)data[1] - '0');
				//pitchAngleTarget += 8.0f;
			}else if(data[0] == 'D'){
				IOPool_pGetWriteData(upperIOPool)->pitchAdd = - 8.0f * ((int)data[1] - '0');
				//pitchAngleTarget -= 8.0f;
			}else if(data[0] == 'T'){
				fw_printfln("received T: %d", data[1]);
			}else if(data[0] == '1'){
				if(data[1] == 'U'){
					flUpDown = 5;
				}else if(data[1] == 'D'){
					flUpDown = -5;
				}else if(data[1] == 'S'){
					flUpDown = 0;
				}
			}else if(data[0] == '2'){
				if(data[1] == 'U'){
					frUpDown = 5;
				}else if(data[1] == 'D'){
					frUpDown = -5;
				}else if(data[1] == 'S'){
					frUpDown = 0;
				}
			}else if(data[0] == '3'){
				if(data[1] == 'U'){
					blUpDown = 5;
				}else if(data[1] == 'D'){
					blUpDown = -5;
				}else if(data[1] == 'S'){
					blUpDown = 0;
				}
			}else if(data[0] == '4'){
				if(data[1] == 'U'){
					brUpDown = 5;
				}else if(data[1] == 'D'){
					brUpDown = -5;
				}else if(data[1] == 'S'){
					brUpDown = 0;
				}
			}else if(data[0] == '5'){
				if(data[1] == 'U'){
					flUpDown = 50;
					frUpDown = 50;
					blUpDown = 50;
					brUpDown = 50;
				}else if(data[1] == 'D'){
					flUpDown = -50;
					frUpDown = -50;
					blUpDown = -50;
					brUpDown = -50;
				}else if(data[1] == 'S'){
					flUpDown = 0;
					frUpDown = 0;
					blUpDown = 0;
					brUpDown = 0;
				}
			}else if(data[0] == '6'){
				if(data[1] == 'U'){
					flUpDown = 50;
					frUpDown = 50;
				}else if(data[1] == 'D'){
					flUpDown = -50;
					frUpDown = -50;
				}
			}else if(data[0] == '7'){
				if(data[1] == 'U'){
					blUpDown = 50;
					brUpDown = 50;
				}else if(data[1] == 'D'){
					blUpDown = -50;
					brUpDown = -50;
				}
			}else if(data[0] == '?'){
				forPidDebug = 1;
			}else if(data[0] == '!'){
				forPidDebug = 0;
			}
			
			IOPool_getNextWrite(upperIOPool);
			
//			else if(data[0] == 'F'){
//				uint32_t temp;
//				if(data[1] == '1'){
//					temp = 1;
//				}else if(data[1] == '0'){
//					temp = 0;
//				}else{
//					temp = 99;
//				}
//				STMFLASH_Write(PARAM_SAVED_START_ADDRESS, &temp, 1);
//				fw_printfln("F: %d", temp);
//			}else if(data[0] == 'X'){
//				uint32_t temp;
//				STMFLASH_Read(PARAM_SAVED_START_ADDRESS, &temp, 1);
//				fw_printfln("Read: %d", temp);
//			}				
			
//			fw_printf("d:");
//			fw_printf("%c|", data[0]);
//			fw_printf("%c|", data[1]);
//			fw_printf("%c|", data[2]);
//			fw_printf("%c|", data[3]);
//			fw_printf("%c|", data[4]);
//			fw_printf("%c|", data[5]);
//			fw_printf("%c|", data[6]);
//			fw_printf("%c|", data[7]);
//			fw_printf("%c|", data[8]);
//			fw_printf("%c\r\n", data[9]);
//			fw_printf("===========\r\n");

		}
	}
}
