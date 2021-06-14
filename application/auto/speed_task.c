#include "sys.h"
#include "shoot.h"
#include "dbus.h"
#include "event_mgr.h"
#include "event.h"
#include "os_timer.h"
#include "speed_task.h"
#include "log.h"

struct rc_device speed_rc;
static void speed_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);

void speed_task(void const *argument)
{
	subscriber_t listSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, speed_dr16_data_update);
	
	rc_device_register(&speed_rc, "Speed RC");
	
	while (1)
	{
		EventMsgProcess(&listSubs, 0);
		
		if (rc_device_get_state(&speed_rc, RC_S1_UP) == E_OK)
        {
            //log_printf("RC_S1_UP");
        }
		else if (rc_device_get_state(&speed_rc, RC_S1_MID) == E_OK)
		{
			//log_printf("RC_S1_MID");
		}
		else if (rc_device_get_state(&speed_rc, RC_S1_DOWN) == E_OK)
		{
			//log_printf("RC_S1_DOWN");
		}
		
		osDelay(5);
	}
}


static void speed_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&speed_rc, pMsgData);
}
