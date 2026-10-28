#include "WiredlessPS2_gamepad.h"
#include "FreeRTOS.h"
#include "task.h"

__weak void WiredlessPS2_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
//	printf("%d,%d\r\n",keyid,event);
}

static GamePadKeyStateType_t WiredlessPS2_GetKeyState(uint8_t bit);

//无线PS2手柄
GamePadType_t Wiredless_USB_PS2Gamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 0,
	.SetVibration = NULL,
	.getKeyState = WiredlessPS2_GetKeyState
};

//按键值
static uint16_t GamePad_KeyOriginalVal = 0;

//获取手柄键值状态函数
static GamePadKeyStateType_t WiredlessPS2_GetKeyState(uint8_t bit)
{
	if( (GamePad_KeyOriginalVal>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}


//定义按键检测事件的各个按键值
static GamePad_CheckEventType_t GamePadKeyCheckEvent[16] = { 0 };


//标志位设置函数,用于辅助ps2手柄解码
static void ps2_set_bit(uint16_t* state,uint8_t state_bit,uint8_t bit)
{
	if(state_bit==1) //指定的位(bit)设置为1,其他位不变
	{
		*state |= (1U<<bit);
	}
	else //指定的位(bit)设置为0,其他位不变
	{
		*state &= ~(1U<<bit);
	}
}

//数据解码
void Wiredless_PS2_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	(void)phost;
	
	uint8_t tmp_bool = 0;
	
	if( gamepad_brand == PS2_USB_Wiredless2 )
	{
		uint8_t rm_val = 0;
		if( buffer[6]==0&&buffer[7]==0 ) rm_val=128;
		else rm_val = buffer[6];
		Wiredless_USB_PS2Gamepad.LX = rm_val;
		
		if( buffer[8]==0&&buffer[9]==0  ) rm_val=128;
		else rm_val = buffer[8];
		Wiredless_USB_PS2Gamepad.LY = 255 - rm_val;
		
		if( buffer[10]==0&&buffer[11]==0  ) rm_val=128;
		else rm_val = buffer[10];
		Wiredless_USB_PS2Gamepad.RX = rm_val;
		
		if( buffer[12]==0&&buffer[13]==0  ) rm_val=128;
		else rm_val = buffer[12];
		Wiredless_USB_PS2Gamepad.RY = 255 - rm_val;
		
		//buffer[2]
		//Rm    Lm    select   start    →      ←       ↓        ↑
		//0		0		0		0		0		0		0		0
		tmp_bool = (buffer[2]>>0)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,4); //↑
		
		tmp_bool = (buffer[2]>>3)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,5); //→
		
		tmp_bool = (buffer[2]>>1)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,6); //↓
		
		tmp_bool = (buffer[2]>>2)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,7); //←
		
		tmp_bool = (buffer[2]>>5)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,0); //seltec key 选择按键	
		
		tmp_bool = (buffer[2]>>4)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,3); //start key 选择按键
		
		tmp_bool = (buffer[2]>>6)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,1); //左摇杆按键	
		
		tmp_bool = (buffer[2]>>7)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,2); //右摇杆按键
		
		tmp_bool = (buffer[3]>>0)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,10); //左扳机1号
		
		tmp_bool = (buffer[3]>>1)&0x01;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,11); //右扳机1号
		
		if(buffer[4]==0xff) tmp_bool=1;
		else tmp_bool=0;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,8); //左扳机2号
		
		if(buffer[5]==0xff) tmp_bool=1;
		else tmp_bool=0;
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,9); //右扳机2号
		
		tmp_bool = (buffer[3]>>4)&0x01;//BLUE
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,14);
		
		tmp_bool = (buffer[3]>>5)&0x01;//RED
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,13);
		
		tmp_bool = (buffer[3]>>6)&0x01;//PINK
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,15);
		
		tmp_bool = (buffer[3]>>7)&0x01;//GREEN
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,12);
	}
	else if( gamepad_brand == PS2_USB_Wiredless1 )
	{
		Wiredless_USB_PS2Gamepad.LX = buffer[3];
		Wiredless_USB_PS2Gamepad.LY = buffer[4];
		Wiredless_USB_PS2Gamepad.RX = buffer[5];
		Wiredless_USB_PS2Gamepad.RY = buffer[6];
		
		tmp_bool = (buffer[1]>>0)&0x01;//select
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,0); //seltec key 选择按键	
		
		tmp_bool = (buffer[1]>>1)&0x01;//start
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,3); //start key 选择按键
		
		tmp_bool = (buffer[1]>>2)&0x01;//Lm
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,1); //左摇杆按键	
		
		tmp_bool = (buffer[1]>>3)&0x01;//Rm
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,2); //右摇杆按键
		
		tmp_bool = (buffer[0]>>4)&0x01;//L1
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,10); //左扳机1号
		
		tmp_bool = (buffer[0]>>5)&0x01;//R1
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,11); //右扳机1号
		
		tmp_bool = (buffer[0]>>6)&0x01;//L2
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,8); //左扳机2号
		
		tmp_bool = (buffer[0]>>7)&0x01;//R2
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,9); //左扳机2号
		
		tmp_bool = (buffer[0]>>0)&0x01;//GREEN
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,12);
		
		tmp_bool = (buffer[0]>>1)&0x01;//RED
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,13);
		
		tmp_bool = (buffer[0]>>2)&0x01;//BLUE
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,14);
		
		tmp_bool = (buffer[0]>>3)&0x01;//PINK
		ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,15);
		
		tmp_bool = buffer[2]&0x0F;//取出低4位
		if(tmp_bool==0x0F)//没有任何按键按下
		{
			ps2_set_bit(&GamePad_KeyOriginalVal,0,4); //↑
			ps2_set_bit(&GamePad_KeyOriginalVal,0,5); //→
			ps2_set_bit(&GamePad_KeyOriginalVal,0,6); //↓
			ps2_set_bit(&GamePad_KeyOriginalVal,0,7); //←
		}
		else if( (tmp_bool&0x01)==0 )
		{	
			switch ((tmp_bool>>1)&0x03)
			{
				case 0x00://↑
					ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //↑
					break;
				case 0x01://→
					ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //→
					break;
				case 0x02://↓
					ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //↓
					break;
				case 0x03://←
					ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //←
					break;
				default:
					break;
			}
		}
		else if( (tmp_bool&0x01)==1 ) //首位为1,代表存在左盘2个按键按下的情况
		{
			switch ((tmp_bool>>1)&0x03)
			{
				case 0x00://↑→
					ps2_set_bit(&GamePad_KeyOriginalVal,1,4);//↑
					ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //→
					break;
				case 0x01://↓→
					ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //↓
					ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //→
					break;
				case 0x02://↓←
					ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //↓
					ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //←
					break;
				case 0x03://↑←
					ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //↑
					ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //←
					break;
				default:
					break;
			}
		}	
	}
	
	//按键回调函数触发
	for (uint8_t key = PS2KEY_SELECT; key <= PS2KEY_4PINK; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(GamePad_KeyOriginalVal,
                                		&GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//触发回调函数
		WiredlessPS2_KeyEvent_Callback(key,event);
	}

}



