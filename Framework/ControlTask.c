#include "ControlTask.h"

WorkState_e workState = PREPARE_STATE;
WorkState_e GetWorkState()
{
	return workState;
}
